#!/usr/bin/env python3
"""Create lossless test frames from saved Gazebo images, with explicit pixel transforms."""
import argparse
import hashlib
import json
from pathlib import Path
import shutil

import cv2
import numpy as np

from arche_experiments.vision import Decoder, Observation, TARGET_ID

ALT_ID = "JANUS-DOCK-99"
SEED = 1127
WIDTH, HEIGHT = 640, 480
COUNT = 12


def sha256(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def image(path):
    pixels = cv2.imread(str(path), cv2.IMREAD_COLOR)
    if pixels is None or pixels.shape != (HEIGHT, WIDTH, 3):
        raise ValueError(f"Expected 640 × 480 BGR pixels: {path}")
    return pixels


def tone(pixels, gain, offset, seed=None, noise_sigma=0):
    rgb = gain * pixels.astype(np.float32) + offset
    if noise_sigma:
        rng = np.random.default_rng(seed)
        rgb += rng.normal(0, noise_sigma, (HEIGHT, WIDTH, 1)).astype(np.float32)
    return np.clip(np.rint(rgb), 0, 255).astype(np.uint8)


def reflection(pixels, center, radii, angle, peak):
    y, x = np.ogrid[:HEIGHT, :WIDTH]
    a = np.deg2rad(angle)
    u, v = x - center[0], y - center[1]
    major = np.cos(a) * u + np.sin(a) * v
    minor = -np.sin(a) * u + np.cos(a) * v
    mask = peak * np.exp(-0.5 * ((major / radii[0]) ** 2 + (minor / radii[1]) ** 2))
    return np.clip(np.rint(pixels + mask[..., None]), 0, 255).astype(np.uint8)


def blur(pixels, kernel_px):
    kernel = np.full((1, kernel_px), 1.0 / kernel_px, dtype=np.float32)
    return cv2.filter2D(pixels, -1, kernel, borderType=cv2.BORDER_REFLECT_101)


def occlude(pixels, rectangle, bgr):
    x0, y0, x1, y1 = rectangle
    result = pixels.copy()
    result[y0:y1, x0:x1] = bgr
    return result


def magnify(pixels, scale, center_u):
    affine = np.array([[scale, 0, center_u - scale * 320],
                       [0, scale, 240 - scale * 240]], dtype=np.float32)
    result = cv2.warpAffine(pixels, affine, (WIDTH, HEIGHT), flags=cv2.INTER_LINEAR,
                            borderMode=cv2.BORDER_CONSTANT, borderValue=(185, 185, 185))
    return result, affine


def image_stats(pixels, roi):
    gray = cv2.cvtColor(pixels, cv2.COLOR_BGR2GRAY)
    x0, y0, x1, y1 = roi
    crop = gray[y0:y1, x0:x1]
    low, high = np.percentile(crop, [5, 95])
    return {"mean_luma_8bit": float(gray.mean()),
            "roi_luma_p05_8bit": float(low), "roi_luma_p95_8bit": float(high),
            "roi_luma_span_p95_p05": float(high - low),
            "roi_clipped_white_fraction": float(np.mean(crop == 255)),
            "roi_laplacian_variance": float(cv2.Laplacian(crop, cv2.CV_64F).var()),
            "quality_roi_xyxy_exclusive": list(roi)}


def inside(corners, rect):
    x0, y0, x1, y1 = rect
    return [bool(x0 <= u < x1 and y0 <= v < y1) for u, v in corners]


def pixel_roi(corners, default=(282, 201, 360, 279)):
    if corners is None:
        return default
    points = np.asarray(corners)
    lower = np.maximum(points.min(axis=0).astype(int), [0, 0])
    upper = np.minimum(np.ceil(points.max(axis=0)).astype(int), [WIDTH, HEIGHT])
    return tuple(map(int, (*lower, *upper)))


def source_record(path, original, capture=None):
    return {"original_path": str(path.relative_to(original)), "source_png_sha256": sha256(path),
            "native_capture": capture}


def build(original, output):
    output.mkdir(parents=True, exist_ok=True)
    clean_capture = original / "native/capture/clean_native"
    capture_rows = json.loads((clean_capture / "capture.json").read_text())
    sources = [(image(clean_capture / item["png"]), clean_capture / item["png"], item)
               for item in capture_rows]
    reference = sources[0][0]
    # Corner annotations use the unobstructed reference, never the disturbed detector output.
    nominal = Decoder().decode(reference, Observation(0.0, 0.0, WIDTH, HEIGHT, TARGET_ID, 320.0))
    if not nominal.valid:
        raise RuntimeError(f"Clean source RGB does not decode: {nominal.reason}")
    corners = np.asarray(nominal.corners_px)
    listings = {}

    def add(name, frames, model, expected_valid, expected_candidates=None):
        folder = output / name
        folder.mkdir()
        frames_info = []
        for i, (pixels, locator, annotation, copy_native, parameters) in enumerate(frames):
            path = folder / f"{i:03d}.png"
            if copy_native is not None:
                shutil.copyfile(copy_native, path)
            elif not cv2.imwrite(str(path), pixels):
                raise OSError(path)
            if not np.array_equal(image(path), pixels):
                raise ValueError(f"PNG pixels changed: {path}")
            roi = pixel_roi(annotation.get("geometric_corners_px"))
            frames_info.append({"frame_index": i, "path": str(path.relative_to(output)),
                                "png_sha256": sha256(path), **locator, **annotation,
                                "actual_parameters": parameters, "image_quality": image_stats(pixels, roi)})
        listings[name] = {"transform": model, "expected_valid": expected_valid,
                          "expected_candidates": expected_candidates, "frames": frames_info}

    def native(name, source, expected, candidates, x_m, y_m):
        directory = original / f"native/capture/{source}"
        captured = json.loads((directory / "capture.json").read_text())
        frames = []
        for item in captured:
            path = directory / item["png"]
            if sha256(path) != item["png_sha256"]:
                raise ValueError(f"Capture digest disagrees: {path}")
            # Station is frontoparallel, body is held at the origin apart from contact settle.
            # Use the validated camera intrinsics and nominal 21/29 active area of the QR sign.
            pose = item["pose_latest_on_callback"]
            fx, _, ux, _, fy, vy, *_ = item["camera_info"]["matrix"]
            half = 0.12 * 21 / 29
            distance = x_m - (pose["x_m"] + 0.25)
            x_cam = y_m - pose["y_m"]
            z_cam = 0.31 - (pose["z_m"] + 0.235)
            projected = [[ux - fx * (x_cam + half) / distance, vy - fy * (z_cam + half) / distance],
                         [ux - fx * (x_cam - half) / distance, vy - fy * (z_cam + half) / distance],
                         [ux - fx * (x_cam - half) / distance, vy - fy * (z_cam - half) / distance],
                         [ux - fx * (x_cam + half) / distance, vy - fy * (z_cam - half) / distance]]
            visibility = inside(projected, (0, 0, WIDTH, HEIGHT))
            annotation = {"geometric_corners_px": projected, "geometric_corners_visible": visibility,
                          "geometric_visible_corner_count": sum(visibility),
                          "annotation_origin": "station pose + native intrinsics, nominal frontoparallel model"}
            frames.append((image(path), source_record(path, original, item), annotation, path,
                           {"station_x_m": x_m, "station_y_m": y_m}))
        add(name, frames, {"kind": "unmodified native RGB", "scene": source}, expected, candidates)

    def transform(name, fn, definition, expected, geometric=None, occlusion=None, repeat=COUNT):
        frames = []
        annotated = corners if geometric is None else geometric
        visibility = inside(annotated, (0, 0, WIDTH, HEIGHT))
        if occlusion is not None:
            visibility = [visible and not obscured for visible, obscured in
                          zip(visibility, inside(annotated, occlusion))]
        for i in range(repeat):
            source, original_png, capture = sources[i % len(sources)]
            pixels, parameters = fn(source, i)
            frames.append((pixels, source_record(original_png, original, capture),
                           {"geometric_corners_px": annotated.tolist(),
                            "geometric_corners_visible": visibility,
                            "geometric_visible_corner_count": sum(visibility),
                            "annotation_origin": "clean reference quadrilateral carried through the declared transform"},
                           None, parameters))
        add(name, frames, definition, expected)

    native("two_ids", "mixed_ids", True, [TARGET_ID, ALT_ID], 1.50, 0.0)
    native("duplicate_ids", "duplicate_ids", False, [TARGET_ID, TARGET_ID], 1.50, 0.0)
    native("native_near_edge", "near_edge", False, [], 0.61, -0.23)
    native("clean", "clean_native", True, [TARGET_ID], 1.30, 0.0)
    prepatch = original / "prepatch/rendered-camera.png"
    add("prepatch_control", [(image(prepatch), source_record(prepatch, original), {}, prepatch, {})],
        {"kind": "unmodified native RGB", "exact_png_capture_stamp": None,
         "scope": "prepatch holdout retained regardless of decoder outcome"}, None)

    for name, definition, expected in (
        ("mild_glare", {"center": [349, 218], "radii": [8, 4], "angle_deg": -30, "peak_dn": 95}, None),
        ("hotspot", {"center": [340, 223], "radii": [34, 28], "angle_deg": 0, "peak_dn": 410}, False),
        ("reflection", {"center": [321, 226], "radii": [57, 15], "angle_deg": -24, "peak_dn": 530}, False),
    ):
        d = definition
        transform(name, lambda source, _: (reflection(source, d["center"], d["radii"],
                                               d["angle_deg"], d["peak_dn"]), d),
                  {"kind": "additive clipped Gaussian specular lobe", **d}, expected)
    for name, gain, offset, expected in (
        ("exposure_mild", 1.65, 32, None),
        ("exposure_clip", 2.20, 235, False),
        ("exposure_whiteout", 1.0, 255, False),
        ("low_light", 0.18, 5, True),
    ):
        definition = {"kind": "linear gain + offset + clamp", "gain": gain, "offset_dn": offset}
        transform(name, lambda source, _: (tone(source, gain, offset), definition), definition, expected)
    for name, kernel, expected in (("motion_blur_mild", 5, None), ("motion_blur", 43, False)):
        definition = {"kind": "horizontal normalized box PSF", "kernel_px": kernel,
                      "border": "reflect_101"}
        transform(name, lambda source, _: (blur(source, kernel), definition), definition, expected)
    dark_blur = {"kind": "linear dimming, correlated luma noise, then horizontal blur",
                 "gain": 0.022, "offset_dn": 2, "sigma_dn": 2.8, "kernel_px": 43, "seed_base": SEED}
    transform("low_light_motion_blur",
              lambda source, i: (blur(tone(source, 0.022, 2, SEED + i, 2.8), 43),
                         {**dark_blur, "seed": SEED + i}), dark_blur, False, repeat=COUNT)

    for name, rect, expected in (("occlusion_mild", (313, 225, 325, 243), None),
                                 ("occlusion", (319, 194, 376, 287), False)):
        definition = {"kind": "opaque mask", "rectangle_xyxy_exclusive": list(rect), "bgr": [100, 105, 111]}
        transform(name, lambda source, _: (occlude(source, rect, definition["bgr"]), definition),
                  definition, expected, occlusion=rect)
    for name, scale, u, expected in (("near_full_view", 2.4, 320, True), ("edge_crop", 2.4, 647, False)):
        _, affine = magnify(reference, scale, u)
        geometric = corners @ affine[:, :2].T + affine[:, 2]
        definition = {"kind": "digital magnification + native-size frame boundary",
                      "scale": scale, "target_center_u_px": u, "affine": affine.tolist()}
        transform(name, lambda source, _: (magnify(source, scale, u)[0], definition),
                  definition, expected, geometric=geometric)

    (output / "manifest.json").write_text(json.dumps({"source": str(original), "seed": SEED,
        "width_px": WIDTH, "height_px": HEIGHT, "opencv": cv2.__version__, "fixtures": listings},
        indent=2, allow_nan=False) + "\n")


if __name__ == "__main__":
    cli = argparse.ArgumentParser(description=__doc__)
    cli.add_argument("original", type=Path)
    cli.add_argument("output", type=Path)
    args = cli.parse_args()
    cv2.setNumThreads(1)
    build(args.original, args.output)
