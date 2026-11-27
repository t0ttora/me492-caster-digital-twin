"""Generate a case-local SDF from the integration robot and world."""
import argparse
import json
import math
from pathlib import Path
import os
import xml.etree.ElementTree as ET

WALLS = {"M07": [-.34, .74, -.36, .76],
         "O02": [-.35, .70, -.34, .34],
         "O03": [-.32, .91, -.34, .34],
         "Q14": [-.36, 1.81, -.34, .34]}
CONTACT_LINKS = ("base_link", "left_wheel", "right_wheel", "caster_wheel")
SEED = 492
THICKNESS = .08


def element(parent, name, value):
    ET.SubElement(parent, name).text = str(value)


def vector(values):
    return " ".join(format(value, ".15g") for value in values)


def box(world, name, center, size, *, pitch=0., mu=.6, color="0.32 0.38 0.43 1"):
    model = ET.SubElement(world, "model", name=name)
    element(model, "static", "true")
    element(model, "pose", vector((*center, 0., pitch, 0.)))
    link = ET.SubElement(model, "link", name="fixture")
    for kind in ("collision", "visual"):
        block = ET.SubElement(link, kind, name=f"box_{kind}")
        geometry = ET.SubElement(block, "geometry")
        element(ET.SubElement(geometry, "box"), "size", vector(size))
        if kind == "collision":
            ode = ET.SubElement(ET.SubElement(ET.SubElement(block, "surface"), "friction"), "ode")
            element(ode, "mu", mu)
            element(ode, "mu2", mu)
        else:
            element(ET.SubElement(block, "material"), "diffuse", color)
    return {"name": name, "center_m": list(center), "size_m": list(size),
            "pitch_rad": pitch, "sdf_mu": mu}


def walls(world, bounds, end_walls=True):
    x0, x1, y0, y1 = bounds
    t, h = THICKNESS, .45
    x, y = (x0+x1)/2, (y0+y1)/2
    spans = [("wall_south", (x, y0-t/2, h/2), (x1-x0+2*t, t, h)),
             ("wall_north", (x, y1+t/2, h/2), (x1-x0+2*t, t, h))]
    if end_walls:
        spans += [("wall_west", (x0-t/2, y, h/2), (t, y1-y0, h)),
                  ("wall_east", (x1+t/2, y, h/2), (t, y1-y0, h))]
    return [box(world, name, center, size) for name, center, size in spans]


def slab(world, name, x0, x1, top=0., **options):
    return box(world, name, ((x0+x1)/2, 0., top-.05), (x1-x0, 3., .10), **options)


def stepped_surface(world, step_height):
    # The pitched top face rises continuously from x=.30 to x=1.00.
    ramp_start, ramp_end = .30, 1.00
    slope, thickness = math.radians(3.), .10
    length = (ramp_end-ramp_start) / math.cos(slope)
    rise = (ramp_end-ramp_start) * math.tan(slope)
    center = ((ramp_start+ramp_end)/2 + thickness*math.sin(slope)/2,
              0., rise/2-thickness*math.cos(slope)/2)
    return [
        slab(world, "approach", -4., ramp_start),
        box(world, "finite_ramp", center, (length, 3., thickness), pitch=-slope,
            color="0.50 0.50 0.56 1"),
        slab(world, "landing", ramp_end, 4., rise),
        box(world, "finite_threshold", (1.64, 0., rise+step_height/2),
            (.24, 1.0, step_height), color="0.73 0.44 0.16 1"),
    ]


def build(source, out, case, variant):
    world_xml = ET.parse(source / "worlds/janus.sdf")
    world = world_xml.getroot().find("world")
    world_name = f"janus_geometry_{case.lower()}"
    world.set("name", world_name)
    spawn = [0., 0., .077, 0., 0., 0.]
    if case == "M07":
        translations = {"original": (0., 0.), "translated-original": (.05, -.05),
                        "replanned": (.0484, -.008)}
        spawn[:2] = translations[variant]
    rendered = case in {"Q14", "O03"}
    # Wall pacing leaves capture-age, acquisition and physics-step limits intact.
    target_rtf = 1.
    if case == "Q14":
        target_rtf = .12 if variant.startswith("qr-runtime") else .055
    world.find("physics/real_time_factor").text = str(target_rtf)
    for plugin in world.findall("plugin"):
        if "Sensors" in plugin.get("name") and not rendered:
            world.remove(plugin)
    ET.SubElement(world, "plugin", filename="gz-sim-contact-system", name="gz::sim::systems::Contact")

    native_contact_plugin = os.environ.get("JANUS_NATIVE_CONTACT_PLUGIN")
    if native_contact_plugin:
        plugin = ET.SubElement(world, "plugin", filename=native_contact_plugin,
                               name="janus::geometry::RecordContact")
        element(plugin, "output", out / "native_contact_step.csv")

    for include in world.findall("include"):
        world.remove(include)
        if include.findtext("uri") == "model://janus_dock" and case == "Q14":
            dock_dir = source / "models/janus_dock"
            dock = ET.parse(dock_dir / "model.sdf").getroot().find("model")
            element(dock, "pose", include.findtext("pose"))
            dock.find(".//mesh/uri").text = (dock_dir / "meshes/marker.dae").resolve().as_uri()
            world.append(dock)

    robot = ET.parse(source / "models/arche_janus/model.sdf").getroot().find("model")
    robot.insert(0, ET.Element("pose"))
    robot.find("pose").text = vector(spawn)
    monitored = {}
    for link in robot.findall("link"):
        name = link.get("name")
        if not rendered:
            for sensor in link.findall("sensor"):
                link.remove(sensor)
        if name not in CONTACT_LINKS:
            continue
        collision = link.find("collision").get("name")
        sensor = ET.SubElement(link, "sensor", name="fixture_contact", type="contact")
        element(sensor, "always_on", "true")
        detail = ET.SubElement(sensor, "contact")
        element(detail, "collision", collision)
        topic = f"/geometry/contact/{name}"
        element(detail, "topic", topic)
        monitored[name] = topic
    world.append(robot)

    fixture_boxes = walls(world, WALLS[case]) if case in WALLS else []
    if case == "M11":
        world.remove(world.find("model[@name='ground']"))
        coefficients = {"boundary": (.60, .12), "uniform-nominal": (.60, .60),
                        "uniform-low": (.12, .12)}[variant]
        fixture_boxes.extend([
            slab(world, "surface_a", -4., .30, mu=coefficients[0], color="0.64 0.65 0.65 1"),
            slab(world, "surface_b", .30, 4., mu=coefficients[1], color="0.46 0.52 0.64 1"),
        ])
    if case == "M12":
        world.remove(world.find("model[@name='ground']"))
        height = {"step05": .005, "step20": .020, "step40": .040}[variant]
        fixture_boxes.extend(stepped_surface(world, height))
    waypoints = None
    if case in {"O03", "Q14"}:
        mover_plugin = os.environ.get("JANUS_NATIVE_MOVER_PLUGIN")
        if not mover_plugin:
            raise RuntimeError("Native crossing scene requires the Gazebo motion plugin")
        waypoints = ([(1.4, -.53), (2.5, .53)] if case == "O03" else
                     [(2., .70), (2.26, 0.), (3.70, 0.), (4., .70)])
        mover = ET.SubElement(world, "plugin", filename=mover_plugin,
                              name="janus::geometry::MoveFixture")
        element(mover, "output", out / "fixture_native_pose.csv")
        for time_s, y in waypoints:
            step = ET.SubElement(mover, "waypoint")
            element(step, "time_s", time_s)
            element(step, "y_m", y)
        fixture_boxes.append(box(world, "crossing_fixture",
                                  (.58, -.53, .35) if case == "O03" else (.81, .70, .20),
                                  (.12, .20, .70) if case == "O03" else (.12, .35, .22),
                                  color="0.83 0.32 0.09 1"))
    bridge = (source / "config/bridge.yaml").read_text().replace("/world/janus/", f"/world/{world_name}/")
    (out / "bridge.yaml").write_text(bridge)
    for visual in world.iter("visual"):
        visual.set("name", visual.get("name") + "_visual")
    ET.indent(world_xml)
    world_xml.write(out / "scene.sdf", encoding="utf-8", xml_declaration=True)
    (out / "fixture.json").write_text(json.dumps({
        "case": case, "variant": variant, "seed": SEED, "world_name": world_name,
        "spawn_pose_xyz_rpy": spawn, "world_step_s": .001, "ground_truth_period_s": .020,
        "rendered_camera": rendered, "bounds_m": WALLS.get(case), "boxes": fixture_boxes,
        "contacts": monitored, "controller_pose_source": "Gazebo world odometry",
        "camera_setup": "integration model parameters" if rendered else "camera disabled to isolate physics",
        "direct_contact_step_instrumentation": native_contact_plugin is not None,
        "gazebo_target_realtime_factor": target_rtf,
        "native_crossing_waypoints_time_y": waypoints,
        "scope": "synthetic laboratory Gazebo scene",
    }, indent=2) + "\n")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("source", type=Path)
    parser.add_argument("output", type=Path)
    parser.add_argument("case", choices=("M07", "M11", "M12", "O02", "O03", "Q14"))
    parser.add_argument("variant")
    args = parser.parse_args()
    build(args.source, args.output, args.case, args.variant)


if __name__ == "__main__":
    main()
