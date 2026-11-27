"""Inject archived pixel-fixture recipes into the sampled Python control loop."""
from dataclasses import asdict
import json
from pathlib import Path
from time import perf_counter

from qr_camera import Candidate, DOCK_ID, Frame


class ImageReplay:
    DELAY_S = .04

    def __init__(self, case, variant):
        from janus.ros2.src.arche_experiments.arche_experiments import fixtures, vision

        _, _, self.image, self.target_u, self.expected_valid, _, _ = next(
            item for item in fixtures.qr_fixtures() if item[:2] == (case, variant))
        self.decoder = vision.Decoder()
        self.make_observation = vision.Observation
        self.cv2 = vision.cv2
        self.source_files = [Path(fixtures.__file__), Path(vision.__file__)]
        self.variant = variant
        self.visible_scene_corners = 2 if variant in {"occlusion", "cropped"} else None
        self.rows = []

    def capture(self, capture_s, camera):
        arrival_s = round(capture_s+self.DELAY_S, 9)
        observation = self.make_observation(capture_s, arrival_s, camera.width_px, camera.height_px,
                                            DOCK_ID, self.target_u, source="synthetic_image_replay")
        started = perf_counter()
        decoded = self.decoder.decode(self.image, observation)
        elapsed = perf_counter()-started
        candidates = tuple(
            Candidate(item.id, item.corners_px,
                      all(0 <= u < camera.width_px and 0 <= v < camera.height_px for u, v in item.corners_px),
                      0. if item.id == decoded.id and decoded.reason == "invalid_geometry" else 1.)
            for item in decoded.candidates)
        self.rows.append({"capture_time_s": capture_s, "arrival_time_s": arrival_s,
                          "decode_wall_s": elapsed, "decoder_valid": decoded.valid,
                          "decoder_reason": decoded.reason, "selected_id": decoded.id,
                          "candidate_count": len(candidates),
                          "scene_visible_corners": self.visible_scene_corners,
                          "candidates_json": json.dumps([asdict(item) for item in decoded.candidates])})
        return Frame(capture_s, arrival_s, candidates, source=decoded.source)

    def png(self):
        success, encoded = self.cv2.imencode(".png", self.image)
        if not success:
            raise RuntimeError("Could not encode image replay fixture")
        return encoded.tobytes()

    def configuration(self):
        return {"input_variant": self.variant, "synthetic_arrival_delay_s": self.DELAY_S,
                "pixel_source": "deterministic ROS lane fixture recipe",
                "opencv": self.cv2.__version__, "fixture_expected_decoder_valid": self.expected_valid,
                "scene_visible_corners_from_fixture_transform": self.visible_scene_corners}
