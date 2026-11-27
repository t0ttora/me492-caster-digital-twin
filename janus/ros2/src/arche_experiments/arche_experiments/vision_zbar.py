"""Optional native ZBar decoding with the common projective geometry policy."""
from math import atan2

import cv2
import numpy as np
from pyzbar.pyzbar import ZBarSymbol, decode

from .vision import Candidate, Decoder


UPRIGHT = {
    'UP': ((1, 0), (0, 1)),
    'LEFT': ((0, -1), (1, 0)),
    'DOWN': ((-1, 0), (0, -1)),
    'RIGHT': ((0, 1), (-1, 0)),
}


def physical_corners(polygon, orientation):
    """Turn a contour hull into the marker's upright TL, TR, BR, BL ordering."""
    if len(polygon) != 4 or orientation not in UPRIGHT:
        raise ValueError('invalid ZBar quadrangle or orientation')
    points = np.asarray(polygon, dtype=np.float32)
    upright = (points-points.mean(axis=0)) @ np.asarray(UPRIGHT[orientation]).T
    order = sorted(range(4),key=lambda n: atan2(upright[n,1],upright[n,0]))
    top_left = min(range(4),key=lambda n: sum(upright[order[n]]))
    indices = order[top_left:] + order[:top_left]
    return points[indices]


class ZBarDecoder(Decoder):
    """Decode actual pixels, and retain the OpenCV check for unknown candidates."""

    def __init__(self, known_single_marker_fixture=False):
        super().__init__(check_residual=not known_single_marker_fixture)

    def _candidates(self, image, frame):
        candidates = []
        for symbol in decode(image, symbols=[ZBarSymbol.QRCODE]):
            try:
                corners = physical_corners(symbol.polygon, symbol.orientation)
                candidate = Candidate.from_points(symbol.data.decode('utf-8'), corners,
                                                  frame.width_px, frame.height_px)
            except ValueError:
                continue
            candidates.append(candidate)
        if not candidates and self.check_residual:
            return super()._candidates(image, frame)
        if len(candidates) > 1 and self.check_residual:
            # Keep unresolved QR geometry visible to the ambiguity contract.
            residual = image.copy()
            fill = int(image.max())
            for candidate in candidates:
                polygon = np.array(candidate.corners_px)
                center = polygon.mean(axis=0)
                expanded = np.rint(center + (polygon-center)*1.15).astype(np.int32)
                cv2.fillConvexPoly(residual, expanded, fill)
            candidates.extend(super()._candidates(residual, frame))
        return candidates
