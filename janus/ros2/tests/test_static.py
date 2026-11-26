import ast
import json
import math
from pathlib import Path
import unittest
import xml.etree.ElementTree as ET

import numpy as np
import xacro
import yaml

from arche_experiments.ros_fields import JOINTS

ROOT = Path(__file__).resolve().parents[1]
SRC = ROOT / "src"
SIM = SRC / "arche_sim"


def floats(value):
    return tuple(map(float, value.split()))


class StaticTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.sdf = ET.parse(SIM / "models/arche_janus/model.sdf").getroot().find("model")
        cls.world = ET.parse(SIM / "worlds/janus.sdf").getroot().find("world")
        cls.config = json.loads((SIM / "config/robot_config.json").read_text())
        xml = xacro.process_file(str(SRC / "arche_description/urdf/arche.urdf.xacro"))
        cls.urdf = ET.fromstring(xml.toxml())

    def test_all_source_xml_yaml_python_parse(self):
        for path in SRC.rglob("*"):
            if not path.is_file():
                continue
            with self.subTest(path=path):
                if path.suffix in (".xml", ".sdf", ".config", ".dae", ".xacro"):
                    ET.parse(path)
                elif path.suffix == ".yaml":
                    self.assertTrue(yaml.safe_load(path.read_text()))
                elif path.suffix == ".py" or (path.parent.name == "scripts" and not path.suffix):
                    ast.parse(path.read_text())
        ast.parse((ROOT / "bench.py").read_text())

    def test_packages_install_all_entrypoints(self):
        packages = {ET.parse(path).getroot().findtext("name") for path in SRC.glob("*/package.xml")}
        self.assertEqual(packages, {"arche_description", "arche_sim", "arche_bringup", "arche_experiments"})
        cmake = (SRC / "arche_experiments/CMakeLists.txt").read_text()
        for script in ("qr_observer", "command_guard", "replay_velocity", "export_bag"):
            self.assertIn(f"scripts/{script}", cmake)
            self.assertTrue((SRC / "arche_experiments/scripts" / script).stat().st_mode & 0o111)

    def test_description_and_sim_joint_graph_agree(self):
        sdf = {joint.attrib["name"]: joint for joint in self.sdf.findall("joint")}
        urdf = {joint.attrib["name"]: joint for joint in self.urdf.findall("joint")
                if joint.attrib["type"] != "fixed"}
        self.assertEqual(set(sdf), set(urdf))
        self.assertEqual(set(sdf), set(JOINTS))
        for name, joint in sdf.items():
            equivalent = urdf[name]
            self.assertEqual(equivalent.find("parent").attrib["link"], joint.findtext("parent"))
            child = equivalent.find("child").attrib["link"]
            self.assertEqual(child, joint.findtext("child"))
            self.assertEqual(floats(equivalent.find("axis").attrib["xyz"]), floats(joint.findtext("axis/xyz")))
            offset = self.sdf.find(f"link[@name='{child}']/pose")
            self.assertEqual(offset.attrib["relative_to"], joint.findtext("parent"))
            self.assertEqual(floats(equivalent.find("origin").attrib["xyz"]), floats(offset.text)[:3])
        plugin = self.sdf.find("plugin[@name='gz::sim::systems::DiffDrive']")
        self.assertEqual(plugin.findtext("left_joint"), "left_wheel_joint")
        self.assertEqual(plugin.findtext("right_joint"), "right_wheel_joint")
        self.assertAlmostEqual(float(plugin.findtext("wheel_separation")), self.config["drive_track_m"])
        self.assertAlmostEqual(float(plugin.findtext("wheel_radius")), self.config["drive_radius_m"])
        self.assertEqual(self.sdf.attrib["canonical_link"], self.config["base_frame"])

    def test_caster_contact_heights_and_camera_transform(self):
        includes = {item.findtext("uri"): item for item in self.world.findall("include")}
        robot = floats(includes["model://arche_janus"].findtext("pose"))
        self.assertEqual(robot[:2], (0, 0))
        wheel = floats(self.sdf.findtext("link[@name='caster_wheel']/pose"))
        caster_low = robot[2] + self.config["caster_pivot_from_axle_m"][2] + wheel[2] - self.config["caster_radius_m"]
        self.assertAlmostEqual(caster_low, robot[2] - self.config["drive_radius_m"])
        self.assertAlmostEqual(-wheel[0], self.config["caster_trail_m"])
        camera = self.config["camera"]
        sensor = self.sdf.find("link[@name='base_link']/sensor")
        self.assertEqual(floats(sensor.findtext("pose"))[:3], tuple(camera["xyz_from_base_link_m"]))
        self.assertEqual(floats(self.urdf.find("joint[@name='camera_mount_joint']/origin").attrib["xyz"]),
                         tuple(camera["xyz_from_base_link_m"]))
        rpy = floats(self.urdf.find("joint[@name='camera_optical_joint']/origin").attrib["rpy"])
        self.assertEqual(rpy, (-math.pi / 2, 0, -math.pi / 2))
        self.assertEqual(camera["frame"], sensor.findtext("camera/optical_frame_id"))
        self.assertEqual(int(sensor.findtext("camera/image/width")), camera["width_px"])
        self.assertEqual(int(sensor.findtext("camera/image/height")), camera["height_px"])
        self.assertEqual(floats(includes["model://janus_dock"].findtext("pose"))[:3],
                         tuple(self.config["marker"]["center_in_map_m"]))

    def test_bridge_topics_authority_and_time(self):
        config = {}
        for name in ("bridge.yaml", "camera_bridge.yaml"):
            for bridge in yaml.safe_load((SIM / "config" / name).read_text()):
                ros = bridge["ros_topic_name"]
                self.assertNotIn(ros, config)
                config[ros] = bridge
        self.assertEqual(config["/clock"]["gz_topic_name"], f"/world/{self.world.attrib['name']}/clock")
        self.assertEqual([k for k, v in config.items() if v["direction"] == "ROS_TO_GZ"], ["/cmd_vel"])
        source_topics = {item.text for item in self.sdf.iter() if item.tag in
                         ("topic", "odom_topic", "tf_topic", "camera_info_topic")}
        self.assertEqual(source_topics, {item["gz_topic_name"] for topic, item in config.items()
                                         if topic != "/clock"})
        diff = self.sdf.find("plugin[@name='gz::sim::systems::DiffDrive']")
        truth = self.sdf.find("plugin[@name='gz::sim::systems::OdometryPublisher']")
        self.assertEqual(diff.findtext("frame_id"), "odom")
        self.assertEqual(truth.findtext("odom_frame"), "map")
        self.assertEqual(diff.findtext("child_frame_id"), truth.findtext("robot_base_frame"))

    def test_world_model_uris_and_marker_uv(self):
        for item in self.world.findall("include"):
            name = item.findtext("uri").removeprefix("model://")
            model = SIM / "models" / name
            relative = ET.parse(model / "model.config").getroot().findtext("sdf")
            self.assertTrue((model / relative).is_file())
        mesh = SIM / "models/janus_dock/meshes/marker.dae"
        dae = ET.parse(mesh).getroot()
        ns = {"c": "http://www.collada.org/2005/11/COLLADASchema"}
        self.assertEqual(dae.findtext(".//c:library_images/c:image/c:init_from", namespaces=ns), "marker.png")
        uv = floats(dae.findtext(".//c:float_array[@id='uv']", namespaces=ns))
        self.assertEqual(uv, (0, 1, 1, 1, 1, 0, 0, 0))
        position = floats(dae.findtext(".//c:float_array[@id='pos']", namespaces=ns))
        self.assertAlmostEqual(max(position) - min(position), self.config["marker"]["textured_placard_edge_m"])

    def test_marker_triangles_face_configured_front(self):
        mesh = SIM / "models/janus_dock/meshes/marker.dae"
        dae = ET.parse(mesh).getroot()
        positions = np.array(floats(dae.findtext(".//{*}float_array[@id='pos']"))).reshape(-1, 3)
        normals = np.array(floats(dae.findtext(".//{*}float_array[@id='norm']"))).reshape(-1, 3)
        triangles = dae.find(".//{*}triangles")
        offsets = {item.get("semantic"): int(item.get("offset")) for item in triangles.findall("{*}input")}
        index = np.fromstring(triangles.findtext("{*}p"), sep=" ", dtype=int)
        index = index.reshape(int(triangles.get("count")), 3, max(offsets.values()) + 1)
        front = np.array(self.config["marker"]["front_normal_in_map"])
        # The dock include, link and visual are unrotated, so mesh and map normals agree.
        include = self.world.find("include[uri='model://janus_dock']")
        dock = ET.parse(SIM / "models/janus_dock/model.sdf").getroot()
        for item in (include, dock.find("model"), dock.find("model/link"), dock.find("model/link/visual")):
            self.assertEqual(floats(item.findtext("pose", "0 0 0 0 0 0"))[3:], (0, 0, 0))
        for triangle in index:
            a, b, c = positions[triangle[:, offsets["VERTEX"]]]
            winding = np.cross(b - a, c - a)
            self.assertGreater(float(winding @ front), 0)
            np.testing.assert_allclose(winding / np.linalg.norm(winding), front)
            np.testing.assert_allclose(normals[triangle[:, offsets["NORMAL"]]], np.tile(front, (3, 1)))
            np.testing.assert_array_equal(triangle[:, offsets["VERTEX"]], triangle[:, offsets["TEXCOORD"]])


if __name__ == "__main__":
    unittest.main()
