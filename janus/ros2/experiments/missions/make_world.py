"""Construct and record a per-case world using the unchanged baseline plant."""
import argparse
import json
from pathlib import Path
import xml.etree.ElementTree as ET

from contract import CASES, DOCK_CASES, GUIDES, NATIVE_CONTACT_LINKS


def build(case, sim, output):
    source_world = sim / "worlds/janus.sdf"
    world_xml = ET.parse(source_world)
    world = world_xml.find("world")
    world.set("name", "janus_mission_" + case.lower())
    physics = world.find("plugin[@filename='gz-sim-physics-system']")
    ET.SubElement(physics, "engine").text = "gz-physics-dartsim-plugin"
    robot = world.find("include[name='arche']")
    if case in {"Q15", "Q16", "Q18"}:
        robot.find("pose").text = ".76 0 .077 0 0 0"
    elif case == "O07":
        robot.find("pose").text = ".82 0 .077 0 0 0"
    if case not in DOCK_CASES:
        world.remove(world.findall("include")[1])
    else:
        for name, (x, y, width, depth) in GUIDES.items():
            model = ET.SubElement(world, "model", name=name)
            ET.SubElement(model, "static").text = "true"
            ET.SubElement(model, "pose").text = f"{x} {y} .10 0 0 0"
            link = ET.SubElement(model, "link", name="guide")
            for element in ("collision", "visual"):
                shape = ET.SubElement(link, element, name=element)
                geometry = ET.SubElement(shape, "geometry")
                ET.SubElement(ET.SubElement(geometry, "box"), "size").text = f"{width} {depth} .20"
                if element == "visual":
                    ET.SubElement(ET.SubElement(shape, "material"), "diffuse").text = ".30 .34 .40 1"
    if case == "O06":
        model_xml = ET.parse(sim / "models/arche_janus/model.sdf")
        model = model_xml.find("model")
        ET.SubElement(world, "plugin", filename="gz-sim-contact-system", name="gz::sim::systems::Contact")
        for name in NATIVE_CONTACT_LINKS:
            link = model.find(f"link[@name='{name}']")
            sensor = ET.SubElement(link, "sensor", name="janus_lab_floor_contact", type="contact")
            ET.SubElement(sensor, "always_on").text = "true"
            ET.SubElement(sensor, "update_rate").text = "50"
            contact = ET.SubElement(sensor, "contact")
            ET.SubElement(contact, "collision").text = link.find("collision").get("name")
            ET.SubElement(contact, "topic").text = f"/janus/gz/floor_contact/{name}"
        payload = ET.SubElement(model, "link", name="janus_lab_payload")
        ET.SubElement(payload, "pose", relative_to="base_link").text = "-.03 0 .145 0 0 0"
        inertial = ET.SubElement(payload, "inertial")
        ET.SubElement(inertial, "mass").text = "4.0"
        inertia = ET.SubElement(inertial, "inertia")
        # 4 kg rectangular payload, mechanically fixed to the deck.
        for axis, value in (("ixx", .016667), ("iyy", .026667), ("izz", .036667)):
            ET.SubElement(inertia, axis).text = str(value)
        for element in ("collision", "visual"):
            geometry = ET.SubElement(ET.SubElement(payload, element, name=element), "geometry")
            ET.SubElement(ET.SubElement(geometry, "box"), "size").text = ".26 .20 .10"
        joint = ET.SubElement(model, "joint", name="janus_lab_payload_fixed", type="fixed")
        ET.SubElement(joint, "parent").text = "base_link"
        ET.SubElement(joint, "child").text = "janus_lab_payload"
        model.insert(0, ET.Element("pose"))
        model.find("pose").text = robot.findtext("pose")
        ET.indent(model_xml)
        model_xml.write(output / "loaded-arche.sdf", encoding="utf-8", xml_declaration=True)
        world.remove(robot)
        world.append(model)
    world_file = output / "world.sdf"
    ET.indent(world_xml)
    world_xml.write(world_file, encoding="utf-8", xml_declaration=True)
    (output / "world.json").write_text(json.dumps({
        "world": world.get("name"),
        "simulator_physics_requested": physics.findtext("engine"),
        "physics_step_s": float(world.findtext("physics/max_step_size")),
        "physics_target_real_time_factor": float(world.findtext("physics/real_time_factor")),
        "robot_initial_xyz_rpy": [float(v) for v in robot.findtext("pose").split()],
        "milieu": "flat nominal Coulomb SDF",
        "dock_guides_xy_size_si": GUIDES if case in DOCK_CASES else {},
        "floor_contact_links": NATIVE_CONTACT_LINKS if case == "O06" else (),
        "payload_kg": 4.0 if case == "O06" else 0.,
        "payload_xyz_from_base_m": [-.03, 0., .145] if case == "O06" else None,
        "random_seed": 42,
    }, indent=2) + "\n")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("case", choices=CASES)
    parser.add_argument("sim", type=Path)
    parser.add_argument("output", type=Path)
    args = parser.parse_args()
    build(args.case, args.sim, args.output)
