"""QR decoding and timestamp validation shared with the Jazzy camera node."""

from dataclasses import asdict, dataclass, replace
import math

import cv2
import numpy as np

OPTICAL_FRAME = "camera_optical_frame"
TARGET_ID = "JANUS-DOCK-01"


@dataclass(frozen=True)
class Candidate:
    id: str | None
    corners_px: tuple
    center_u_px: float
    center_v_px: float
    area_ratio: float

    @classmethod
    def from_points(cls, text, points, width, height):
        corners = np.asarray(points, dtype=np.float32).reshape(4, 2)
        if not np.isfinite(corners).all():
            raise ValueError("nonfinite QR corners")
        area = cv2.contourArea(corners)
        # Intersection of diagonals is the projection of the physical marker center.
        homogeneous = np.column_stack((corners, np.ones(4)))
        center = np.cross(np.cross(homogeneous[0], homogeneous[2]),
                          np.cross(homogeneous[1], homogeneous[3]))
        if area <= 0 or abs(center[2]) < 1e-9:
            raise ValueError("degenerate QR corners")
        u, v = center[:2] / center[2]
        return cls(text or None, tuple(tuple(map(float, point)) for point in corners),
                   float(u), float(v), area / (width * height))


@dataclass(frozen=True)
class Observation:
    capture_time_s: float
    arrival_time_s: float
    width_px: int
    height_px: int
    target_id: str
    target_u_px: float
    candidates: tuple = ()
    id: str | None = None
    corners_px: tuple = ()
    center_u_px: float | None = None
    center_v_px: float | None = None
    quality_area_ratio: float | None = None
    visible: bool = False
    valid: bool = False
    reason: str = "no_qr"
    source: str = "decoded_image"
    frame_id: str = OPTICAL_FRAME

    @property
    def error(self):
        if self.center_u_px is None:
            return None
        return (self.center_u_px - self.target_u_px) / self.width_px

    def as_row(self):
        return {**asdict(self), "normalized_error": self.error}


def select_marker(frame, candidates, minimum_area=0.0004):
    selection = [candidate for candidate in candidates if candidate.id == frame.target_id]
    frame = replace(frame, candidates=tuple(candidates), visible=bool(candidates))
    if len(selection) > 1:
        return replace(frame, reason="ambiguous_target")
    if not selection:
        reason = "wrong_id" if any(c.id for c in candidates) else "undecoded" if candidates else "no_qr"
        return replace(frame, reason=reason)
    if any(candidate.id is None for candidate in candidates):
        return replace(frame, reason="ambiguous_undecoded")
    marker = selection[0]
    bounded = all(0 <= u < frame.width_px and 0 <= v < frame.height_px for u, v in marker.corners_px)
    valid = (0 <= frame.target_u_px < frame.width_px and bounded
             and marker.area_ratio >= minimum_area)
    return replace(frame, id=marker.id, corners_px=marker.corners_px,
                   center_u_px=marker.center_u_px, center_v_px=marker.center_v_px,
                   quality_area_ratio=marker.area_ratio, valid=valid,
                   reason="accepted" if valid else "invalid_geometry")


class Decoder:
    def __init__(self, check_residual=True):
        self.detector = cv2.QRCodeDetector()
        self.check_residual = check_residual

    def decode(self, image, frame):
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY) if image.ndim == 3 else image
        low, high, _, _ = cv2.minMaxLoc(gray)
        if 0 < high - low < 64:
            gray = cv2.normalize(gray, None, 0, 255, cv2.NORM_MINMAX)
        try:
            candidates = self._candidates(gray, frame)
            if self.check_residual and len(candidates) == 1 and candidates[0].id == frame.target_id:
                # OpenCV's multi detector can collapse two same-size adjacent markers into one.
                # Re-scan the unoccupied pixels before accepting a sole target.
                polygon = np.array(candidates[0].corners_px)
                center = polygon.mean(axis=0)
                polygon = np.rint(center + (polygon - center) * 1.12).astype(np.int32)
                residual = gray.copy()
                cv2.fillConvexPoly(residual, polygon, int(high if high - low >= 64 else 255))
                candidates.extend(self._candidates(residual, frame))
        except cv2.error:
            return replace(frame, reason="decoder_error")
        return select_marker(frame, candidates)

    def _candidates(self, image, frame):
        _, strings, points, _ = self.detector.detectAndDecodeMulti(image)
        if points is None or not len(points):
            text, single, _ = self.detector.detectAndDecode(image)
            strings = (text,)
            points = () if single is None else single.reshape(1, 4, 2)
        candidates = []
        for index, corners in enumerate(points):
            try:
                text = strings[index] if index < len(strings) else ""
                candidate = Candidate.from_points(text, corners, frame.width_px, frame.height_px)
            except ValueError:
                continue
            if (candidate.id is None and candidate.area_ratio >= .0004 and
                    all(0 <= u < frame.width_px and 0 <= v < frame.height_px for u, v in candidate.corners_px)):
                decoded = self._decode_rectified(image, candidate.corners_px)
                if decoded:
                    candidate = replace(candidate, id=decoded)
            candidates.append(candidate)
        return candidates

    def _decode_rectified(self, gray, corners):
        # Rendered modules can lie between input pixels. Binarize after expanding the
        # detected quadrilateral and restore a quiet zone without changing its geometry.
        square = np.float32(((0, 0), (251, 0), (251, 251), (0, 251)))
        transform = cv2.getPerspectiveTransform(np.float32(corners), square)
        patch = cv2.warpPerspective(gray, transform, (252, 252), flags=cv2.INTER_LINEAR)
        padded = cv2.copyMakeBorder(patch, 32, 32, 32, 32, cv2.BORDER_CONSTANT, value=255)
        _, binary = cv2.threshold(padded, 0, 255, cv2.THRESH_BINARY | cv2.THRESH_OTSU)
        return self.detector.detectAndDecode(binary)[0]


class FrameTiming:
    """Strict capture order and expiry in the caller's declared clock domain."""

    def __init__(self, freshness_s=0.20):
        if not math.isfinite(freshness_s) or freshness_s <= 0:
            raise ValueError("freshness_s must be finite and positive")
        self.freshness_s = freshness_s
        self.last_arrival = self.last_capture = None
        self.clock_reset = False

    def accept(self, frame):
        capture, arrival = frame.capture_time_s, frame.arrival_time_s
        if not (math.isfinite(capture) and math.isfinite(arrival)) or min(capture, arrival) < 0:
            return replace(frame, valid=False, reason="invalid_stamp")
        if self.last_arrival is not None and arrival < self.last_arrival:
            self.clock_reset = True
        self.last_arrival = arrival
        if self.clock_reset:
            return replace(frame, valid=False, reason="clock_reset")
        if capture > arrival:
            return replace(frame, valid=False, reason="future_frame")
        if self.last_capture is not None and capture <= self.last_capture:
            cause = "duplicate_frame" if capture == self.last_capture else "out_of_order_frame"
            return replace(frame, valid=False, reason=cause)
        self.last_capture = capture
        if arrival - capture > self.freshness_s:
            return replace(frame, valid=False, reason="stale_frame")
        return frame


class CenteringGate:
    """Offline reference for how a consumer should gate forward motion."""

    def __init__(self):
        self.frame = None
        self.dwell_start = None
        self.forward = False

    def update(self, now, frame=None):
        if self.frame is not None and now - self.frame.capture_time_s > 0.20:
            self.dwell_start = None
            self.forward = False
        if frame is not None:
            self.frame = frame
        if self.frame is None or not self.frame.valid or now - self.frame.capture_time_s > 0.20:
            self.dwell_start = None
            self.forward = False
            return False
        error = abs(self.frame.error)
        if error > 0.04:
            self.forward = False
        if not self.forward and error > 0.02:
            self.dwell_start = None
        elif not self.forward:
            # Start dwell on receipt; a wall timer alone cannot complete it on a held frame.
            if self.dwell_start is None:
                self.dwell_start = self.frame.arrival_time_s
            self.forward = self.frame.arrival_time_s - self.dwell_start >= 0.40
        return self.forward


def unpack_image(data, width, height, step, encoding):
    channels = {"mono8": 1, "rgb8": 3, "bgr8": 3, "rgba8": 4, "bgra8": 4}
    count = channels.get(encoding)
    if count is None or width <= 0 or height <= 0 or step < width * count:
        raise ValueError("unsupported encoding or invalid image dimensions")
    raw = np.frombuffer(data, dtype=np.uint8)
    if len(raw) != height * step:
        raise ValueError("image buffer length disagrees with dimensions and stride")
    img = raw.reshape(height, step)[:, :width * count].reshape(height, width, count)
    conversions = {"rgb8": cv2.COLOR_RGB2BGR, "rgba8": cv2.COLOR_RGBA2BGR, "bgra8": cv2.COLOR_BGRA2BGR}
    if encoding in conversions:
        return cv2.cvtColor(img, conversions[encoding])
    return np.ascontiguousarray(img[:, :, 0] if count == 1 else img)
