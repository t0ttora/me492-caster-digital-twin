"""Record native Gazebo and ROS streams while exercising a laboratory fixture."""
import argparse
import json
import math
from pathlib import Path
import time

from gz.msgs10.double_pb2 import Double
from gz.msgs10.pose_pb2 import Pose
from gz.transport13 import Node as GzNode
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import Image, JointState
from std_msgs.msg import String

from arche_experiments.ros_fields import joint_fields, odometry_fields, stamp_s, twist_fields
from inject import PREFIX
from motion import (STILL_DWELL, advance, clamp, follow, pose_gate, rounded_reference,
                    rounded_segments, segment_reference, terminal, wrap)


class Recorder(Node):
    def __init__(self, directory):
        super().__init__("janus_geometry_recorder")
        self.directory = directory
        self.fixture = json.loads((directory/"fixture.json").read_text())
        self.case = self.fixture["case"]
        self.variant = self.fixture["variant"]
        self.trace = (directory/"trace.jsonl").open("w", buffering=262144)
        self.gz = GzNode()
        self.clock = self.origin = None
        self.start_pose = tuple(self.fixture["spawn_pose_xyz_rpy"][index] for index in (0, 1, 5))
        self.gt = self.pose = None
        self.last_odom = self.last_joint = None
        self.last_tick = -1.
        self.gate_since = None
        self.terminal_at = None
        self.emitted = set()
        self.reason = None
        self.qr = None
        self.center_since = None
        self.camera_frames = 0
        self.fixture_pose = None
        self.fixture_was_occupied = False
        self.crossing = self.case in {"O03", "Q14"}
        self.qos_latest = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.segment = 0
        self.segment_start = 0.
        self.pub = self.create_publisher(Twist, "/janus/cmd_vel/request", 10)
        for msg_type, topic, callback in [
                (Clock, "/clock", self.clock_update),
                (Odometry, "/janus/ground_truth", self.truth),
                (Odometry, "/odom", self.odom),
                (JointState, "/joint_states", self.joint),
                (Twist, "/cmd_vel", self.applied),
                (String, "/janus/command_guard/status", self.guard),
        ]:
            qos = self.qos_latest if msg_type in {Clock, JointState} else qos_profile_sensor_data
            self.create_subscription(msg_type, topic, callback, qos)
        if self.crossing:
            self.gz.subscribe(Pose, "/geometry/fixture_pose", self.fixture_position)
            self.scene_start = self.gz.advertise("/geometry/scene_start", Double)
            self.create_subscription(Image, "/janus/camera/image_raw", self.camera,
                                     qos_profile_sensor_data)
            if self.case == "Q14":
                self.create_subscription(String, "/janus/qr/observation", self.qr_update, 10)
            self.start_signal = self.create_publisher(String, f"{PREFIX}/start", 10)
            self.injector_input = self.create_subscription(
                String, f"{PREFIX}/injected_event", self.injected_event, 10)

    def row(self, source, **data):
        self.trace.write(json.dumps({"source": source, "read_time_s": self.clock, **data},
                                    allow_nan=False, separators=(",", ":")) + "\n")

    def event(self, name, **info):
        if name not in self.emitted:
            self.emitted.add(name)
            self.row("event", event=name, **info)

    def truth(self, msg):
        data = odometry_fields(msg, "map")
        q = msg.pose.pose.orientation
        twist = msg.twist.twist
        self.gt = data
        self.last_lateral = twist.linear.y
        self.pose = (data["x_m"], data["y_m"], data["yaw_rad"])
        self.row("ground_truth", **data, quat_xyzw=[q.x, q.y, q.z, q.w],
                 lateral_mps=twist.linear.y, vertical_mps=twist.linear.z,
                 roll_rate_radps=twist.angular.x, pitch_rate_radps=twist.angular.y)

    def odom(self, msg):
        self.last_odom = stamp_s(msg.header.stamp)
        self.row("odom", **odometry_fields(msg, "odom"))

    def joint(self, msg):
        self.last_joint = stamp_s(msg.header.stamp)
        self.row("joint_states", **joint_fields(msg))

    def guard(self, msg):
        self.row("guard_status", **json.loads(msg.data))

    def applied(self, msg):
        self.row("guard_output", **twist_fields(msg))

    def qr_update(self, msg):
        self.qr = json.loads(msg.data)
        payload = dict(self.qr)
        payload["observation_source"] = payload.pop("source")
        self.row("rendered_qr", **payload)

    def injected_event(self, msg):
        payload = json.loads(msg.data)
        if self.origin is None or payload["case"] != self.case:
            return
        name = {"hazard_present": "obstacle_detected" if self.case == "Q14" else "pedestrian_detected",
                "operator_consent": "authorized_resume"}.get(payload["event"])
        if name:
            self.row("injected_event", **payload)
            self.event(name, producer_time_s=payload["time_s"], provenance=payload["provenance"])

    def camera(self, msg):
        self.camera_frames += 1
        self.row("camera_frame", time_s=stamp_s(msg.header.stamp), width_px=msg.width,
                 height_px=msg.height, encoding=msg.encoding, frame_id=msg.header.frame_id)
        if self.camera_frames == 15 or (self.origin is not None and
                self.clock-self.origin > 2.3 and "obstacle_frame_saved" not in self.emitted):
            import cv2
            from arche_experiments.vision import unpack_image
            image = unpack_image(msg.data, msg.width, msg.height, msg.step, msg.encoding)
            tag = "initial" if self.camera_frames == 15 else "crossing"
            cv2.imwrite(str(self.directory/f"rgb-{tag}.png"), image)
            if tag == "crossing":
                self.event("obstacle_frame_saved", capture_time_s=stamp_s(msg.header.stamp))

    def fixture_position(self, msg):
        self.fixture_pose = (msg.header.stamp.sec + msg.header.stamp.nsec/1e9,
                             msg.position.x, msg.position.y, msg.position.z)

    def qr_command(self):
        if not self.qr or not self.qr["valid"] or self.qr["id"] != "JANUS-DOCK-01":
            self.center_since = None
            return 0., 0.
        age = self.clock - self.qr["capture_time_s"]
        if not 0. <= age <= .20:
            self.center_since = None
            return 0., 0.
        error = (self.qr["center_u_px"]-self.qr["target_u_px"])/self.qr["width_px"]
        if abs(error) > .04:
            self.center_since = None
        if abs(error) <= .02 and self.center_since is None:
            self.center_since = self.clock
        if self.center_since is None or self.clock-self.center_since < .40:
            if abs(error) > .02:
                self.center_since = None
            return 0., clamp(-1.5*error, .25)
        return .13, clamp(-1.5*error, .25)

    def crossing_control(self, elapsed):
        q14 = self.case == "Q14"
        clear = 4. if q14 else 2.5
        blocked = ("obstacle_detected" if q14 else "pedestrian_detected") in self.emitted
        if self.fixture_pose:
            stamp, x, y, z = self.fixture_pose
            self.row("fixture_world_pose", time_s=stamp, x_m=x, y_m=y, z_m=z)
            half_width = self.fixture["boxes"][-1]["size_m"][1]/2
            lane_edge = self.fixture["bounds_m"][3]
            if blocked and self.clock-.10 <= stamp <= self.clock+.02:
                if abs(y)-half_width < lane_edge:
                    self.fixture_was_occupied = True
                if elapsed >= clear and self.fixture_was_occupied and y-half_width >= lane_edge:
                    self.event("fixture_clear_observed", provenance="fresh Gazebo crossing fixture pose")
        if blocked and not {"fixture_clear_observed", "authorized_resume"} <= self.emitted:
            self.center_since = None
            return "blocked_hold", 0., 0., None
        if {"fixture_clear_observed", "authorized_resume"} <= self.emitted:
            self.event("resume_gate_open")
        if self.pose[0] >= (.70 if q14 else .54):
            self.event("final_brake")
            return "final_stop", 0., 0., None
        if q14:
            v, w = self.qr_command()
            return "qr_advance" if v else "qr_wait_or_center", v, w, None
        return ("authorized_transit" if blocked else "crossing_approach",
                .17, clamp(-2.5*self.pose[2]-4.*self.pose[1], .4), None)

    def mark_gate(self, target):
        speed = math.hypot(self.gt["v_mps"], self.last_lateral)
        if pose_gate(self.pose, target, speed, self.gt["yaw_rate_radps"]):
            if self.gate_since is None:
                self.gate_since = self.clock
            return self.clock-self.gate_since >= STILL_DWELL
        self.gate_since = None
        return False

    def control(self, elapsed):
        if self.case in {"O03", "Q14"}:
            return self.crossing_control(elapsed)

        if self.case == "M07":
            segments = rounded_segments(self.variant)
            segment, target, v, w = rounded_reference(self.start_pose, segments, elapsed)
            if segment < len(segments):
                if segment != self.segment:
                    self.segment = segment
                    self.event(f"segment_{segment+1}")
                return f"rounded_segment_{segment+1}", *follow(self.pose, target, v, w), target
            if self.terminal_at is not None:
                return "settled", 0., 0., target
            if self.mark_gate(target):
                self.terminal_at = self.clock
                self.event("terminal_gate_verified", dwell_start_s=self.gate_since)
                return "settled", 0., 0., target
            return "terminal", *terminal(self.pose, target), target

        if self.case == "M11":
            if elapsed >= 4.:
                self.event("brake_step")
                return "brake", 0., 0., (.6, 0., 0.)
            return "surface_crossing", .15, clamp(-2.5*self.pose[2]-4.*self.pose[1], .4), (.15*elapsed, 0., 0.)

        if self.case == "M12":
            if self.pose[0] >= 2.24 or "brake_step" in self.emitted or elapsed >= 22.:
                self.event("brake_step")
                return "brake", 0., 0., (2.24, 0., 0.)
            return "ramp_threshold_transit", .125, clamp(-2.5*self.pose[2]-4.*self.pose[1], .4), None

        # Test the nominal turned egress and, separately, a literal reverse-gear egress.
        if self.variant == "turned-egress":
            stages = ((.35, 0., 2.4), (0., math.pi, 4.), (.35, 0., 2.4))
            names = ("forward_entry", "turn_180", "forward_exit_after_turn")
        else:
            stages = ((.35, 0., 2.4), (0., math.pi, 4.), (0., -math.pi, 4.), (-.35, 0., 4.))
            names = ("forward_entry", "turn_180", "turn_to_reverse_heading", "reverse_exit")
        start = self.start_pose
        for stage in stages[:self.segment]:
            start = advance(start, *stage[:2])
        if self.segment == len(stages):
            return "settled", 0., 0., start
        stage = stages[self.segment]
        end = advance(start, *stage[:2])
        local = elapsed-self.segment_start
        self.event(f"start_{names[self.segment]}", target=end)
        if local < stage[2]:
            target, v, w = segment_reference(start, stage, local)
            return names[self.segment], *follow(self.pose, target, v, w), target
        if self.mark_gate(end):
            self.event(f"gate_{names[self.segment]}", dwell_start_s=self.gate_since)
            self.gate_since = None
            self.segment_start = elapsed
            self.segment += 1
            if self.segment == len(stages):
                self.terminal_at = self.clock
            return "segment_gate", 0., 0., end
        return f"align_{names[self.segment]}", *terminal(self.pose, end, stage[0] < 0), end

    def send(self, v, w):
        message = Twist()
        message.linear.x, message.angular.z = v, w
        self.pub.publish(message)

    def capture_graph(self):
        topics = ["/clock", "/janus/cmd_vel/request", "/cmd_vel", "/odom",
                  "/janus/ground_truth", "/joint_states"]
        if self.crossing:
            topics += ["/janus/camera/image_raw", f"{PREFIX}/injected_event"]
        if self.case == "Q14":
            topics.append("/janus/qr/observation")
        graph = {topic: [endpoint.node_name for endpoint in self.get_publishers_info_by_topic(topic)]
                 for topic in topics}
        (self.directory/"ros_publisher_graph.json").write_text(json.dumps(graph, indent=2)+"\n")

    def clock_update(self, msg):
        self.clock = stamp_s(msg.clock)
        if self.origin is None:
            robot_ready = (self.clock >= .8 and self.gt and self.last_odom and self.last_joint
                           and self.pub.get_subscription_count()
                           and (self.directory/"contact_recorder_ready.txt").exists())
            crossing_ready = (not self.crossing or (self.camera_frames >= 6 and self.fixture_pose
                              and (self.qr is not None or self.case != "Q14")
                              and self.start_signal.get_subscription_count()
                              and self.injector_input.get_publisher_count()))
            if not robot_ready or not crossing_ready:
                return
            self.origin = self.clock
            self.row("event", event="case_start", origin_sim_time_s=self.origin)
            self.capture_graph()
            if self.crossing:
                self.start_signal.publish(String(data=json.dumps(
                    {"case": self.case, "origin_sim_time_s": self.origin})))
        elapsed = self.clock-self.origin
        if self.crossing and elapsed <= .30:
            self.scene_start.publish(Double(data=self.origin))
        if elapsed < self.last_tick + .0195:
            return
        self.last_tick = elapsed
        if self.clock-self.gt["time_s"] > .12:
            self.send(0., 0.)
            self.row("request", v_mps=0., yaw_rate_radps=0., phase="ground_truth_stale", reference=None)
            return
        phase, v, w, target = self.control(elapsed)
        self.send(v, w)
        self.row("request", phase=phase, v_mps=v, yaw_rate_radps=w, reference=target)
        end_time = {"M07": 30., "M11": 6.5, "M12": 24., "O02": 37., "O03": 9., "Q14": 11.}[self.case]
        if elapsed >= end_time or (self.terminal_at is not None and self.clock-self.terminal_at >= 1.):
            self.reason = "terminal_gate_and_hold" if self.terminal_at else "fixture_time_budget"

    def stop(self):
        if self.crossing:
            self.gz.unsubscribe("/geometry/fixture_pose")
        self.trace.close()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("out", type=Path)
    args = parser.parse_args()
    rclpy.init()
    node = Recorder(args.out)
    start = time.monotonic()
    try:
        while rclpy.ok() and not node.reason:
            rclpy.spin_once(node, timeout_sec=.05)
            if node.origin is None and time.monotonic()-start > 45.:
                node.reason = "stream_setup_timeout"
    except KeyboardInterrupt:
        node.reason = "interrupted"
    except Exception as error:
        node.reason = f"controller_error:{type(error).__name__}"
        raise
    finally:
        if rclpy.ok():
            node.send(0., 0.)
        node.row("event", event="case_end", reason=node.reason)
        (args.out/"termination.json").write_text(json.dumps({
            "origin_sim_time_s": node.origin, "end_sim_time_s": node.clock,
            "reason": node.reason, "camera_frames": node.camera_frames,
            "wall_duration_s": time.monotonic()-start,
        }, indent=2)+"\n")
        node.stop()
        node.destroy_node()
        rclpy.try_shutdown()
    if node.origin is None:
        raise SystemExit(2)


if __name__ == "__main__":
    main()
