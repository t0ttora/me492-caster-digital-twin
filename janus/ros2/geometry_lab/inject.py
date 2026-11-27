"""Standalone ROS producer for scripted lab hazard and operator messages."""
import argparse
import json

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rosgraph_msgs.msg import Clock
from std_msgs.msg import String

from arche_experiments.ros_fields import stamp_s

PREFIX = "/janus/geometry_lab"
SCHEDULE = {"O03": (1.4, 3.), "Q14": (2., 4.8)}


class LabEvents(Node):
    def __init__(self, case, variant):
        super().__init__("janus_geometry_fixture_events")
        self.case = case
        self.origin = None
        self.events = []
        hazard, authorise = SCHEDULE[case]
        self.events.append((hazard, "hazard_present", "scripted lab hazard input"))
        if not variant.endswith("no-authority"):
            self.events.append((authorise if not variant.endswith("early-authority") else hazard+.35,
                                "operator_consent", "scripted lab operator stand-in"))
        self.pub = self.create_publisher(String, f"{PREFIX}/injected_event", 10)
        self.create_subscription(String, f"{PREFIX}/start", self.start, 10)
        self.create_subscription(Clock, "/clock", self.step, qos_profile_sensor_data)

    def start(self, msg):
        data = json.loads(msg.data)
        if self.origin is None and self.case == data["case"]:
            self.origin = data["origin_sim_time_s"]

    def step(self, msg):
        t = stamp_s(msg.clock)
        if self.origin is None:
            return
        while self.events and t-self.origin >= self.events[0][0]-1e-8:
            candidate, name, provenance = self.events.pop(0)
            data = {"case": self.case, "event": name, "scheduled_offset_s": candidate,
                    "time_s": t, "provenance": provenance}
            self.pub.publish(String(data=json.dumps(data)))
            print(json.dumps(data), flush=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("case", choices=SCHEDULE)
    parser.add_argument("variant")
    args = parser.parse_args()
    rclpy.init()
    node = LabEvents(args.case, args.variant)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
