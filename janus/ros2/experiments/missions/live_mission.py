"""Drive one isolated native Gazebo mission and log observable ROS traffic."""
import argparse
from collections import Counter
from dataclasses import fields
import json
import math
from pathlib import Path
import time

import cv2
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rosgraph_msgs.msg import Clock
from ros_gz_interfaces.msg import Contacts
from sensor_msgs.msg import CameraInfo, Image, JointState
from std_msgs.msg import String

from arche_experiments.ros_fields import joint_fields, odometry_fields, stamp_s, twist_fields
from arche_experiments.vision import CenteringGate, Observation, unpack_image
from contract import (
    BMS_FRESHNESS_S, CASES, CHARGE_WAIT_S, CONTACT_RELEASE_M, DOCK_CASES, DOCK_TARGET,
    LOW_SOC_LIMIT, NATIVE_CONTACT_LINKS, QR_FRESHNESS_S, QR_ID, QR_SIDE_M, QR_STANDOFF_M,
    SERVICE_TARGET, STOP_DWELL_S, TRANSIT_TARGET, UNDOCK_TARGET_X, WAYPOINTS,
    fresh_bms, in_position, stopped, wrap, zero,
)

OBSERVATION_FIELDS = {field.name for field in fields(Observation)}
CONTROL_PERIOD_S = .025
SENSOR_AGE_S = .12
SENSOR_LEAD_S = .025
LAB = "/janus/lab/"


def payload(msg):
    return json.loads(msg.data)


def publish_json(publisher, row):
    publisher.publish(String(data=json.dumps(row, allow_nan=False)))


class VirtualStation(Node):
    """Lab-only fixture. Publishes explicitly synthetic readings over ROS."""
    def __init__(self, case):
        super().__init__("janus_lab_station")
        self.case = case
        self.now = None
        self.truth = None
        self.events = {}
        self.sent = set()
        self.sequence = 0
        self.outputs = {name: self.create_publisher(String, LAB + name, 10)
                        for name in ("contact", "bms", "soc")}
        self.create_subscription(String, LAB + "events", self.on_event, 10)
        self.create_subscription(Odometry, "/janus/ground_truth", self.on_truth, qos_profile_sensor_data)
        self.create_subscription(Clock, "/clock", self.on_clock, qos_profile_sensor_data)

    def on_event(self, msg):
        row = payload(msg)
        self.events[row["event"]] = row["time_s"]

    def on_truth(self, msg):
        self.truth = odometry_fields(msg, "map")

    def on_clock(self, msg):
        self.now = stamp_s(msg.clock)

    def after(self, name, delay):
        return name in self.events and self.now - self.events[name] >= delay

    def send_once(self, topic, stimulus, value, age=0.):
        if stimulus in self.sent:
            return
        self.sent.add(stimulus)
        self.sequence += 1
        publish_json(self.outputs[topic], {
            "origin": "virtual_station_fixture", "stimulus": stimulus, "value": value,
            "sequence": self.sequence, "capture_time_s": self.now - age,
        })

    def step(self):
        if self.now is None or self.truth is None:
            return
        truth_fresh = -SENSOR_LEAD_S <= self.now - self.truth["time_s"] <= SENSOR_AGE_S
        if self.case == "O07" and self.after("transit_started", 0.):
            self.send_once("soc", "nominal_transit_soc", .55)
            if self.after("transit_started", 1.40):
                self.send_once("soc", "low_soc", .12)
        if self.after("case_start", .25) and self.case in {"Q16", "Q18", "O07"}:
            self.send_once("bms", "precontact_charge", True)
        if self.after("zero_motion", .10) and truth_fresh and in_position(self.truth, DOCK_TARGET):
            self.send_once("contact", "virtual_dock_contact", True)
        if self.after("wait_for_charge", .36) and self.case in {"Q16", "Q18", "O07"}:
            self.send_once("bms", "expired_charge", True, age=BMS_FRESHNESS_S + .12)
            if self.after("wait_for_charge", .50):
                self.send_once("bms", "fresh_charge", True)
        if self.case == "Q18" and self.after("charge_cut_request", .20):
            self.send_once("bms", "charge_cut", False)
        if (self.case == "Q18" and self.after("reverse_enabled", 0.) and truth_fresh
                and self.truth["x_m"] <= DOCK_TARGET[0] - CONTACT_RELEASE_M):
            self.send_once("contact", "virtual_contact_release", False)


class Mission(Node):
    def __init__(self, case, run_dir, trace):
        super().__init__("janus_live_mission")
        self.case = case
        self.out = run_dir
        self.trace = trace
        self.counts = Counter()
        self.events = {}
        self.now = None
        self.phase = "initial"
        self.status = None
        self.latest = {}
        self.signals = {}
        self.frame = None
        self.intrinsics = None
        self.gate = CenteringGate()
        self.stop_since = None
        self.next_sample = None
        self.image_s = -1.
        self.invalid_image_s = -5.
        self.save_next_image = "initial"
        self.image_pending = set()
        self.image_cache = {}
        self.last_clock_sample = -1.
        self.last_joint_sample = -1.
        self.route_index = 0
        self.last_contact_sample = {}
        self.goal_open = None
        self.transit_soc = None
        self.publish_request = self.create_publisher(Twist, "/janus/cmd_vel/request", 10)
        self.publish_event = self.create_publisher(String, LAB + "events", 10)
        for cls, topic, func in (
            (Clock, "/clock", self.clock), (Odometry, "/janus/ground_truth", self.truth),
            (Odometry, "/odom", self.odom), (JointState, "/joint_states", self.joints),
            (Twist, "/cmd_vel", self.guarded), (String, "/janus/command_guard/status", self.guard_status),
            (Image, "/janus/camera/image_raw", self.image), (CameraInfo, "/janus/camera/camera_info", self.info),
            (String, "/janus/qr/observation", self.qr),
        ):
            self.create_subscription(cls, topic, func, qos_profile_sensor_data)
        for name in ("contact", "bms", "soc"):
            self.create_subscription(String, LAB + name, lambda msg, source=name: self.signal(source, msg), 10)
        if case == "O06":
            for link in NATIVE_CONTACT_LINKS:
                self.create_subscription(Contacts, LAB + "floor_contact/" + link,
                                         lambda msg, wheel=link: self.floor_contact(wheel, msg),
                                         qos_profile_sensor_data)

    def write(self, kind, **row):
        self.counts[kind] += 1
        self.trace.write(json.dumps({"source": kind, "read_time_s": self.now, **row}, allow_nan=False) + "\n")

    def clock(self, msg):
        stamp = stamp_s(msg.clock)
        if self.now is not None and stamp < self.now:
            raise ValueError("simulation clock reset during the mission")
        self.now = stamp
        if stamp - self.last_clock_sample >= .020:
            self.last_clock_sample = stamp
            self.write("clock", time_s=stamp)

    def event(self, name, **details):
        row = {"event": name, "time_s": self.now, **details}
        self.events[name] = self.now
        self.write("event", **row)
        publish_json(self.publish_event, row)

    def truth(self, msg):
        self.state("truth", odometry_fields(msg, "map"))

    def odom(self, msg):
        self.state("odom", odometry_fields(msg, "odom"))

    def joints(self, msg):
        row = joint_fields(msg)
        self.latest["joint"] = row
        if row["time_s"] - self.last_joint_sample >= .020:
            self.last_joint_sample = row["time_s"]
            self.write("joint", **row)

    def state(self, kind, row):
        self.latest[kind] = row
        self.write(kind, **row)

    def guarded(self, msg):
        self.state("guard_output", {"time_s": self.now, **twist_fields(msg)})

    def guard_status(self, msg):
        self.write("guard_status", **payload(msg))

    def image(self, msg):
        if self.case not in DOCK_CASES:
            return
        stamp = stamp_s(msg.header.stamp)
        self.image_cache[stamp] = msg
        for old_stamp in list(self.image_cache):
            if stamp - old_stamp > .40:
                del self.image_cache[old_stamp]
        if self.save_next_image is not None:
            self.save_image(msg, self.save_next_image)
            self.save_next_image = None
        if stamp in self.image_pending:
            self.save_image(msg, f"decoded-{stamp:.3f}")
            self.image_pending.discard(stamp)

    def save_image(self, msg, label):
        stamp = stamp_s(msg.header.stamp)
        pixels = unpack_image(msg.data, msg.width, msg.height, msg.step, msg.encoding)
        filename = label + "-rendered.png"
        if not cv2.imwrite(str(self.out / filename), pixels):
            raise OSError(f"could not save {filename}")
        self.write("image", file=filename, capture_time_s=stamp, width_px=msg.width, height_px=msg.height,
                   frame_id=msg.header.frame_id)

    def info(self, msg):
        if self.intrinsics is None and msg.k[0] > 0.:
            self.intrinsics = msg.k[0]
            self.write("intrinsics", matrix=list(msg.k), frame_id=msg.header.frame_id,
                       width_px=msg.width, height_px=msg.height)

    def qr(self, msg):
        row = payload(msg)
        self.frame = Observation(**{key: row[key] for key in OBSERVATION_FIELDS if key in row})
        row["observation_source"] = row.pop("source")
        self.write("qr", **row)
        stamp = self.frame.capture_time_s
        if self.frame.valid and self.frame.id == QR_ID and stamp - self.image_s >= 1.:
            self.image_s = stamp
            if stamp in self.image_cache:
                self.save_image(self.image_cache[stamp], f"decoded-{stamp:.3f}")
            else:
                self.image_pending.add(stamp)
        elif not self.frame.valid and stamp - self.invalid_image_s >= 5.:
            self.invalid_image_s = stamp
            if stamp in self.image_cache:
                self.save_image(self.image_cache[stamp], f"rejected-{stamp:.3f}")

    def floor_contact(self, wheel, msg):
        t = stamp_s(msg.header.stamp)
        if t-self.last_contact_sample.get(wheel, -1.) < .02 or not msg.contacts:
            return
        self.last_contact_sample[wheel] = t
        self.write("floor_contact", time_s=t, wheel=wheel, pairs=[
            {"collision_1": pair.collision1.name, "collision_2": pair.collision2.name,
             "points": len(pair.positions),
             "force_z_body_1_N": sum(w.body_1_wrench.force.z for w in pair.wrenches),
             "force_z_body_2_N": sum(w.body_2_wrench.force.z for w in pair.wrenches),
             "depths_m": list(pair.depths)}
            for pair in msg.contacts
        ])

    def signal(self, name, msg):
        row = payload(msg)
        self.signals[name] = row
        self.write(name, **row)

    def command(self, v=0., yaw=0., **details):
        cmd = Twist()
        cmd.linear.x, cmd.angular.z = v, yaw
        self.publish_request.publish(cmd)
        self.write("request", time_s=self.now, phase=self.phase, mission_id=self.goal_open,
                   **twist_fields(cmd), **details)

    def finish(self, status):
        self.phase = "terminal"
        self.status = status
        self.event("terminal", outcome=status)
        self.command()

    def fresh_state(self):
        return (self.now is not None and {"truth", "odom", "joint", "guard_output"} <= self.latest.keys()
                and all(row["time_s"] is not None
                        and -SENSOR_LEAD_S <= self.now - row["time_s"] <= SENSOR_AGE_S
                        for row in self.latest.values()))

    def held(self, goal):
        body, odom, joints = (self.latest[name] for name in ("truth", "odom", "joint"))
        return in_position(body, goal) and stopped(body, odom, joints) and zero(self.latest["guard_output"])

    def dwell(self, good):
        if not good:
            self.stop_since = None
        elif self.stop_since is None:
            self.stop_since = self.now
        return self.stop_since is not None and self.now - self.stop_since >= STOP_DWELL_S

    def waypoint_command(self, goal, speed=.12):
        # Stage/route control consumes wheel odometry. Independent world pose is only
        # used for the terminal measurement against the fixture's predeclared target.
        ox, oy, _ = self.initial_world_from_odom
        estimate = self.latest["odom"]
        dx, dy = goal[0] - (estimate["x_m"]+ox), goal[1] - (estimate["y_m"]+oy)
        x_err = dx * math.cos(goal[2]) + dy * math.sin(goal[2])
        yaw_err = wrap(goal[2] - estimate["yaw_rad"])
        if abs(x_err) <= .006 and abs(yaw_err) <= .012:
            self.command()
            return
        v = max(-speed, min(speed, 1.20*x_err))
        yaw = max(-.22, min(.22, 1.8*yaw_err))
        self.command(v, yaw)

    def route(self):
        if self.case == "O01":
            route = WAYPOINTS
        else:
            route = (("service-park", SERVICE_TARGET),)
        mission_id, goal = route[self.route_index]
        if self.goal_open is None:
            self.stop_since = None
            self.goal_open = mission_id
            self.event("mission_reset", mission_id=mission_id, previous_goal_cleared=True,
                       prior_stop_dwell_s=0., charge_state="idle")
            self.event("goal_started", mission_id=mission_id, target_xyz=goal)
        self.waypoint_command(goal)
        if self.dwell(self.held(goal)):
            self.event("arrived", mission_id=mission_id, target_xyz=goal)
            self.goal_open = None
            self.route_index += 1
            if self.route_index == len(route):
                self.finish("route_completed")

    def approach(self):
        frame = self.frame
        usable = (frame is not None and frame.valid and frame.id == QR_ID
                  and 0. <= self.now - frame.capture_time_s <= QR_FRESHNESS_S)
        if not usable:
            # Replace stale/invalid state so a previous frame cannot carry dwell through dropout.
            self.gate = CenteringGate()
            self.command(qr_usable=False)
            return
        if "qr_acquired" not in self.events:
            self.event("qr_acquired", capture_time_s=frame.capture_time_s)
        forward = self.gate.update(self.now, frame)
        yaw = max(-.45, min(.45, -1.8 * frame.error))
        data = {"qr_usable": True, "capture_time_s": frame.capture_time_s, "qr_error": frame.error}
        if not forward or self.intrinsics is None:
            self.command(0., yaw, **data)
            return
        c0, c1, c2, c3 = frame.corners_px
        height_px = .5 * (math.dist(c0, c3) + math.dist(c1, c2))
        depth = self.intrinsics * QR_SIDE_M / height_px
        if depth <= QR_STANDOFF_M:
            self.phase = "braking"
            self.stop_since = None
            self.event("optical_stop", estimated_depth_m=depth, **data)
            image = self.image_cache.get(frame.capture_time_s)
            if image is not None:
                self.save_image(image, "optical-stop")
            else:
                self.save_next_image = "optical-stop"
            self.command(**data)
        else:
            if "qr_advance" not in self.events:
                self.event("qr_advance", **data)
            self.command(min(.12, max(.018, .8*(depth - QR_STANDOFF_M))), yaw, **data)

    def dock(self):
        contact = self.signals.get("contact")
        bms = self.signals.get("bms")
        if self.phase == "approach":
            self.approach()
            if self.now - self.events["dock_started"] >= 32.:
                self.finish("dock_approach_timeout")
            return
        if self.phase == "braking":
            self.command()
            if "aligned" not in self.events and in_position(self.latest["truth"], DOCK_TARGET):
                self.event("aligned", origin="gazebo_world_geometry")
            if self.dwell(self.held(DOCK_TARGET)):
                self.event("zero_motion", origin="guard_and_native_odometry_encoders")
                self.phase = "contact_pending"
            elif self.now - self.events["optical_stop"] >= 1.50:
                self.finish("mechanical_settle_failed")
            return
        if self.phase == "contact_pending":
            self.command()
            if not self.held(DOCK_TARGET):
                self.finish("mechanical_hold_lost")
            elif (contact and contact["value"] is True
                  and fresh_bms(contact["capture_time_s"], self.now, self.events["zero_motion"])):
                self.event("contact", sequence=contact["sequence"], origin=contact["origin"])
                self.event("wait_for_charge")
                self.phase = "waiting"
            elif self.now - self.events["zero_motion"] >= 1.0:
                self.finish("virtual_contact_timeout")
            return
        if self.phase == "waiting":
            self.command()
            wait_start = self.events["wait_for_charge"]
            if not self.held(DOCK_TARGET):
                self.finish("mechanical_hold_lost")
            elif self.now - wait_start >= CHARGE_WAIT_S:
                self.event("charge_timeout", budget_s=CHARGE_WAIT_S)
                self.finish("charge_timeout")
            elif bms and bms["value"] is True and fresh_bms(bms["capture_time_s"], self.now, wait_start):
                self.event("charge_confirmed", sequence=bms["sequence"], origin=bms["origin"])
                self.event("dock_success", sequence=bms["sequence"])
                if self.case == "Q18":
                    self.phase = "charging"
                else:
                    self.finish("virtual_charge_confirmed")
            return
        self.undock(bms, contact)

    def undock(self, bms, contact):
        if self.phase != "reverse":
            self.command()
            if not self.held(DOCK_TARGET):
                self.finish("mechanical_hold_lost")
            elif self.phase == "charging" and self.now - self.events["dock_success"] >= .35:
                self.event("undock_authorized", origin="explicit_lab_fixture_permission")
                self.event("charge_cut_request")
                self.phase = "cut_pending"
            elif self.phase == "cut_pending":
                cut = self.events["charge_cut_request"]
                if bms and bms["value"] is False and fresh_bms(bms["capture_time_s"], self.now, cut):
                    self.event("charge_cut_confirmed", sequence=bms["sequence"], origin=bms["origin"])
                    self.phase = "reverse_ready"
                elif self.now-cut >= 1.50:
                    self.finish("charge_cut_timeout")
            elif self.phase == "reverse_ready":
                self.event("reverse_enabled")
                self.phase = "reverse"
                self.stop_since = None
            return
        goal = (UNDOCK_TARGET_X, 0., 0.)
        elapsed = self.now - self.events["reverse_enabled"]
        if (contact and contact["value"] is False and "contact_released" not in self.events
                and contact["capture_time_s"] > self.events["reverse_enabled"]):
            self.event("contact_released", sequence=contact["sequence"], origin=contact["origin"])
        if elapsed < .35:
            # Small, declared steering seed breaks the perfectly symmetric reversal equilibrium.
            self.command(-.08, .16)
        else:
            self.waypoint_command(goal)
        if self.dwell(self.held(goal)) and "contact_released" in self.events:
            self.event("exit_stopped")
            self.finish("lab_undock_completed")
        elif elapsed >= 18.:
            self.finish("undock_exit_timeout")

    def transit(self):
        if self.phase == "transit":
            soc = self.signals.get("soc")
            if (soc and soc["value"] <= LOW_SOC_LIMIT
                    and 0. <= self.now-soc["capture_time_s"] <= BMS_FRESHNESS_S):
                self.event("soc_below_threshold", sequence=soc["sequence"], limit=LOW_SOC_LIMIT)
                self.event("transit_preempted", prior_mission="nominal_transit", next_mission="return_to_dock")
                self.phase = "handoff"
                self.command()
            else:
                self.waypoint_command(TRANSIT_TARGET)
        elif self.phase == "handoff":
            self.command()
            measured = stopped(*(self.latest[name] for name in ("truth", "odom", "joint")))
            if self.dwell(measured and zero(self.latest["guard_output"])):
                self.event("transit_stopped")
                self.event("dock_started", mission_id="return_to_dock")
                self.phase = "approach"

    def step(self):
        if self.now is None:
            return False
        if self.next_sample is not None and self.now < self.next_sample:
            return False
        self.next_sample = self.now + CONTROL_PERIOD_S
        if self.status is not None:
            self.command()
            return self.now - self.events["terminal"] >= .40
        if not self.fresh_state():
            self.stop_since = None
            self.command()
            return False
        if self.phase == "initial":
            if self.publish_request.get_subscription_count() == 0:
                return False
            if self.case in DOCK_CASES and (self.intrinsics is None or self.frame is None):
                return False
            world, odom = self.latest["truth"], self.latest["odom"]
            self.initial_world_from_odom = (world["x_m"]-odom["x_m"], world["y_m"]-odom["y_m"],
                                            wrap(world["yaw_rad"]-odom["yaw_rad"]))
            self.event("case_start", initial_world_from_odom=self.initial_world_from_odom)
            if self.case == "O07":
                self.phase = "transit"
                self.event("transit_started", mission_id="nominal_transit", target_xyz=TRANSIT_TARGET)
            elif self.case in DOCK_CASES:
                self.phase = "approach"
                self.event("dock_started", mission_id="dock")
            else:
                self.phase = "route"
        if self.case in {"O01", "O06"}:
            self.route()
        elif self.phase in {"transit", "handoff"}:
            self.transit()
        else:
            self.dock()
        return False


def run(case, out, wall_limit):
    rclpy.init()
    with (out / "telemetry.jsonl").open("x") as trace:
        mission = Mission(case, out, trace)
        station = VirtualStation(case) if case in DOCK_CASES else None
        executor = SingleThreadedExecutor()
        executor.add_node(mission)
        if station is not None:
            executor.add_node(station)
        wall_start = time.monotonic()
        ended = False
        try:
            while time.monotonic() - wall_start < wall_limit:
                executor.spin_once(timeout_sec=.005)
                if station is not None:
                    station.step()
                if mission.step():
                    ended = True
                    break
        finally:
            if rclpy.ok():
                mission.command()
            summary = {
                "case_id": case, "fixture_loop_finished": ended, "execution_outcome": mission.status,
                "events_s": mission.events, "samples": dict(mission.counts),
                "clock_end_s": mission.now, "wall_elapsed_s": time.monotonic()-wall_start,
            }
            (out / "execution.json").write_text(json.dumps(summary, indent=2) + "\n")
            print(json.dumps(summary, indent=2))
            executor.shutdown()
            mission.destroy_node()
            if station is not None:
                station.destroy_node()
            rclpy.try_shutdown()
    return 0 if ended else 2


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("case", choices=CASES)
    parser.add_argument("output", type=Path)
    parser.add_argument("--wall-s", type=float, default=125.)
    options = parser.parse_args()
    raise SystemExit(run(options.case, options.output, options.wall_s))
