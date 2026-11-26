"""Exercise actual QR image decoding independently of controller projection."""
import argparse
from datetime import datetime, timezone
from pathlib import Path

from janus_artifacts import atomic_json, digest
from qr_camera import Camera, Candidate, DOCK_ID, Frame
from qr_docking import FrameGate


def decode_fixture(output):
    import cv2
    from PIL import Image
    import qrcode

    other = "JANUS-OTHER-99"

    def tile(payload):
        code = qrcode.QRCode(box_size=7, border=4)
        code.add_data(payload)
        return code.make_image(fill_color="black", back_color="white").convert("RGB")

    output.mkdir(parents=True, exist_ok=False)
    fixtures = {
        "nominal": ((DOCK_ID, 215, 80),),
        "wrong-only": ((other, 90, 72),),
        "two-stations": ((DOCK_ID, 49, 80), (other, 355, 139)),
        "ambiguous-id": ((DOCK_ID, 49, 80), (DOCK_ID, 355, 139)),
        "blank": (),
    }
    detector = cv2.QRCodeDetector()
    camera = Camera()
    results = []
    for name, placements in fixtures.items():
        path = output/f"{name}.png"
        image = Image.new("RGB",(640,480),"white")
        for payload, x, y in placements:
            image.paste(tile(payload), (x,y))
        image.save(path)
        _, payloads, positions, _ = detector.detectAndDecodeMulti(cv2.imread(str(path)))
        expected = sorted(payload for payload, _, _ in placements)
        candidates = tuple(Candidate((payloads[i] or None) if i < len(payloads) else None,
                                     tuple(map(tuple, quad.tolist())), True, 1.)
                           for i, quad in enumerate(positions if positions is not None else ()))
        observation = FrameGate(camera).update(0., [Frame(0., 0., candidates, "decoded_image")])
        passed = (sorted(candidate.id or "" for candidate in candidates) == expected
                  and observation.valid == (expected.count(DOCK_ID) == 1))
        results.append({"fixture":name, "image":path.name, "sha256":digest(path), "expected_ids":expected,
                        "candidates":[{"id":candidate.id, "corners_px":candidate.corners_px,
                                       "center_px":candidate.center_px} for candidate in candidates],
                        "selection": DOCK_ID if observation.valid else None,
                        "selection_reason": observation.reason, "passed":passed})
    report = {"schema_version":"janus-python-1","evidence_scope":"synthetic_image_decode",
              "created_utc":datetime.now(timezone.utc).isoformat(), "opencv_version":cv2.__version__,
              "scope":"Static generated PNGs decoded by OpenCV. Timing, optics and closed-loop camera response are outside this experiment.",
              "checks_passed":all(result["passed"] for result in results),"cases":results}
    atomic_json(output/"decode_report.json",report)
    print(output/"decode_report.json")
    return report["checks_passed"]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output-dir",type=Path,required=True)
    args = parser.parse_args()
    if not decode_fixture(args.output_dir):
        raise SystemExit(1)


if __name__=="__main__":
    main()
