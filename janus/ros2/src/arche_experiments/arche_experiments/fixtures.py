"""Deterministic image and message inputs. Pixel distortions are synthetic."""
from dataclasses import replace

import cv2
import numpy as np
import qrcode

from .commands import CommandGuard
from .vision import CenteringGate, Decoder, FrameTiming, Observation, TARGET_ID

WIDTH, HEIGHT = 640, 480
WRONG_ID = "JANUS-DOCK-99"


def qr(text, pixels=144):
    code = qrcode.QRCode(error_correction=qrcode.constants.ERROR_CORRECT_M, box_size=4, border=4)
    code.add_data(text)
    code.make(fit=True)
    image = np.asarray(code.make_image().convert("L"))
    return cv2.resize(image, (pixels, pixels), interpolation=cv2.INTER_NEAREST)


def canvas(*markers):
    image = np.full((HEIGHT, WIDTH), 185, dtype=np.uint8)
    for mark, u, v in markers:
        half = mark.shape[0] // 2
        image[v - half:v + half, u - half:u + half] = mark
    return image


def qr_fixtures():
    target, other = qr(TARGET_ID), qr(WRONG_ID)
    center = canvas((target, 320, 240))
    blank = canvas()
    occluded = center.copy()
    occluded[190:300, 310:390] = 185
    whiteout = center.copy()
    whiteout[160:320, 240:400] = 255
    glare = center.copy()
    cv2.circle(glare, (348, 217), 6, 255, -1)
    blur_kernel = np.ones((1, 43), dtype=np.float32) / 43
    cropped = np.column_stack((center[:, 292:], np.full((HEIGHT, 292), 185, dtype=np.uint8)))
    yield "Q01", "nominal", center, 320.0, True, "accepted", None
    yield "Q02", "left", canvas((target, 220, 240)), 320.0, True, "accepted", 1
    yield "Q03", "right", canvas((target, 420, 240)), 320.0, True, "accepted", -1
    yield "Q05", "out_of_view", blank, 320.0, False, "no_qr", None
    yield "Q06", "image_loss", blank, 320.0, False, "no_qr", None
    yield "Q07", "reacquired_image", center, 320.0, True, "accepted", None
    yield "Q08", "wrong_id", canvas((other, 320, 240)), 320.0, False, "wrong_id", None
    yield "Q09", "mixed_ids", canvas((target, 180, 240), (other, 460, 240)), 320.0, True, "accepted", 1
    yield "Q09", "duplicate_target", canvas((target, 180, 240), (target, 460, 240)), 320.0, False, "ambiguous_target", None
    yield "Q10", "spot_glare", glare, 320.0, False, "undecoded", None
    yield "Q10", "whiteout", whiteout, 320.0, False, None, None
    yield "Q11", "low_light", (center.astype(np.float32) * 0.18 + 5).astype("uint8"), 320.0, True, "accepted", None
    yield "Q11", "horizontal_motion_blur", cv2.filter2D(center, -1, blur_kernel), 320.0, False, None, None
    yield "Q12", "occlusion", occluded, 320.0, False, None, None
    yield "Q12", "cropped", cropped, 320.0, False, None, None
    yield "Q19", "rotated_shifted_target", canvas((cv2.rotate(target, cv2.ROTATE_90_CLOCKWISE), 375, 240)), 375.0, True, "accepted", None


def timing_fixture():
    ready = Decoder().decode(canvas((qr(TARGET_ID), 320, 240)),
                             Observation(0, 0, WIDTH, HEIGHT, TARGET_ID, 320))
    sequence = [
        (0.00, 0.06, "accepted"),
        (0.00, 0.10, "duplicate_frame"),
        (0.03, 0.30, "stale_frame"),
        (0.02, 0.31, "out_of_order_frame"),
        (0.38, 0.35, "future_frame"),
        (0.35, 0.36, "accepted"),
        (0.39, 0.41, "accepted"),
        (0.00, 0.01, "clock_reset"),
        (0.10, 0.11, "clock_reset"),
    ]
    timing = FrameTiming()
    for capture, arrival, expected in sequence:
        output = timing.accept(replace(ready, capture_time_s=capture, arrival_time_s=arrival,
                                       source="replay"))
        yield {**output.as_row(), "expected_reason": expected, "passed": output.reason == expected}


def centering_fixture():
    gate, timing = CenteringGate(), FrameTiming()
    sequence = [
        (0.00, "centered", False), (0.10, "centered", False),
        (0.20, "centered", False), (0.30, "centered", False),
        (0.40, "centered", True), (0.48, "near_edge", True),
        (0.50, "outside", False), (0.55, "centered", False),
        (0.65, "centered", False), (0.75, "centered", False),
        (0.85, "centered", False), (0.96, "centered", True),
        (0.98, "lost", False), (1.00, "centered", False),
        (1.10, "centered", False), (1.20, "centered", False),
        (1.30, "centered", False), (1.41, "centered", True),
        (1.55, "stream_cut", True), (1.62, "stream_cut", False),
        (1.63, "duplicate", False),
    ]
    latest_capture = None
    for now, stimulus, expected in sequence:
        frame = None
        if stimulus != "stream_cut":
            position = {"near_edge": 340, "outside": 350, "lost": None}.get(stimulus, 320)
            stamp = latest_capture if stimulus == "duplicate" else now
            frame = timing.accept(Observation(stamp, now, WIDTH, HEIGHT, TARGET_ID, 320.0,
                                              center_u_px=position, valid=position is not None,
                                              visible=position is not None,
                                              reason="accepted" if position is not None else "no_qr",
                                              source="replay"))
            latest_capture = stamp
        observed = gate.update(now, frame)
        yield {"time_s": now, "stimulus": stimulus, "capture_time_s": latest_capture,
               "gate_forward_allowed": observed, "expected_forward_allowed": expected,
               "observation_reason": frame.reason if frame else "no_message",
               "passed": observed == expected}


def guard_fixture(variant):
    guard = CommandGuard()
    restart_at = None
    if variant in ("command_timeout", "odom_timeout"):
        steps = [
            (0, 0, True, True, False, "ready"),
            (0.2, 0.2, variant == "command_timeout", variant == "odom_timeout", False, "ready"),
            (0.26, 0.26, variant == "command_timeout", variant == "odom_timeout", False, variant),
        ]
    elif variant == "operator_abort":
        steps = [(0, 0, True, True, False, "ready"), (0.02, 0.02, True, True, True, variant),
                 (0.04, 0.04, True, True, False, variant)]
    elif variant == "clock_reset":
        steps = [(0.4, 0, True, True, False, "ready"), (0, 0.02, False, False, False, variant),
                 (0.1, 0.12, True, True, False, variant)]
    elif variant == "odom_regression":
        steps = [(0.4, 0, True, True, False, "ready"), (0.42, 0.02, True, True, False, variant),
                 (0.44, 0.04, True, True, False, variant)]
    elif variant == "clock_stall":
        steps = [(0.4, 0, True, True, False, "ready"), (0.4, 0.51, False, False, False, variant)]
    elif variant == "clock_recovery":
        steps = [(0.4, 0, True, True, False, "ready"),
                 (0.4, 0.51, False, True, False, "clock_stall"),
                 (0.42, 0.53, True, False, False, "command_timeout"),
                 (0.44, 0.55, True, True, False, "ready")]
    elif variant == "odom_recovery":
        steps = [(0, 0, True, True, False, "ready"),
                 (0.26, 0.26, False, True, False, "odom_timeout"),
                 (0.28, 0.28, True, False, False, "command_timeout"),
                 (0.30, 0.30, True, True, False, "ready")]
    elif variant == "restart_recovery":
        steps = [(0.4, 0, True, True, False, "ready"),
                 (0.42, 0.02, True, False, True, "operator_abort"),
                 (0.44, 0.04, True, True, False, "operator_abort"),
                 (0, 0.60, True, False, False, "command_timeout"),
                 (0.02, 0.62, True, True, False, "ready")]
        restart_at = 3
    else:
        raise ValueError(f"unknown guard fixture: {variant}")
    for step, (time_s, wall, odom, request, abort, expected) in enumerate(steps):
        if step == restart_at:
            guard = CommandGuard()
        guard.clock(time_s, wall)
        if odom:
            stamp = time_s - 0.06 if variant == "odom_regression" and time_s == 0.42 else time_s
            guard.odometry(stamp)
        if request:
            guard.request(0.1, 0.2, wall)
        if abort:
            guard.abort()
        output = guard.output(wall)
        moving = (output.v_mps, output.yaw_rate_radps) == (0.1, 0.2)
        restart_event = {"guard_restarted": step == restart_at} if restart_at is not None else {}
        yield {"time_s": time_s, "wall_s": wall, **restart_event, "odom_sent": odom, "request_sent": request,
               "abort_requested": abort, "requested_v_mps": 0.1 if request else None,
               "requested_yaw_rate_radps": 0.2 if request else None, **vars(output),
               "expected_reason": expected, "passed": output.reason == expected and moving == (expected == "ready")
               and (moving or (output.v_mps, output.yaw_rate_radps) == (0, 0))}
