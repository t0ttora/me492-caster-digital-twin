#!/usr/bin/env python3
"""Stage the source Gazebo model and the station textures used by the qualification."""
import argparse
import hashlib
import json
from pathlib import Path
import shutil
import subprocess
import tempfile
import xml.etree.ElementTree as ET

import qrcode


def sha256(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def write_xml(path, tree):
    ET.indent(tree, space="  ")
    tree.write(path, encoding="utf-8", xml_declaration=True)


def prepare(installation, output):
    models = output / "models"
    evidence = output / "station-inputs"
    evidence.mkdir(parents=True)
    shutil.copytree(installation / "models", models)
    alternate = models / "janus_distractor"
    shutil.copytree(models / "janus_dock", alternate)
    sdf = alternate / "model.sdf"
    sdf.write_text(sdf.read_text().replace("janus_dock", "janus_distractor")
                   .replace("marker.dae", "distractor.dae"))
    mesh = alternate / "meshes/marker.dae"
    mesh.rename(alternate / "meshes/distractor.dae")
    mesh = alternate / "meshes/distractor.dae"
    mesh.write_text(mesh.read_text().replace("marker.png", "distractor.png")
                    .replace("qr-", "other-").replace("placard", "other-placard"))
    (alternate / "meshes/marker.png").unlink()
    config = alternate / "model.config"
    config.write_text(config.read_text().replace("janus_dock", "janus_distractor")
                      .replace("JANUS-DOCK-01", "JANUS-DOCK-99"))
    ids = {"JANUS-DOCK-01": models / "janus_dock/meshes/marker.png",
           "JANUS-DOCK-99": alternate / "meshes/distractor.png"}
    code = qrcode.QRCode(version=1, error_correction=qrcode.constants.ERROR_CORRECT_M,
                         box_size=16, border=4)
    code.add_data("JANUS-DOCK-99")
    code.make(fit=False)
    code.make_image().convert("RGB").save(ids["JANUS-DOCK-99"])

    # Sign the exact input asset bytes independently of the QR application payload.
    manifest = {"scope": "locally signed JANUS simulator fixture inputs",
                "algorithm": "Ed25519 detached PNG signature",
                "trust": "fresh local qualification key; no site trust anchor available",
                "payload_contract": "plain ID string, as accepted by the production decoder",
                "stations": []}
    with tempfile.TemporaryDirectory(dir=output) as temp:
        key = Path(temp) / "key.pem"
        public = evidence / "qualification-public.pem"
        subprocess.run(["openssl", "genpkey", "-algorithm", "ED25519", "-out", key], check=True)
        subprocess.run(["openssl", "pkey", "-in", key, "-pubout", "-out", public], check=True)
        for identity, rendered_path in ids.items():
            asset = evidence / f"{identity}.png"
            shutil.copyfile(rendered_path, asset)
            signature = asset.with_suffix(".png.sig")
            subprocess.run(["openssl", "pkeyutl", "-sign", "-rawin", "-inkey", key,
                            "-in", asset, "-out", signature], check=True)
            subprocess.run(["openssl", "pkeyutl", "-verify", "-pubin", "-rawin", "-inkey", public,
                            "-in", rendered_path, "-sigfile", signature], check=True)
            manifest["stations"].append({"id": identity, "png": asset.name,
                "sha256": sha256(asset), "signature": signature.name,
                "rendered_asset": str(rendered_path.relative_to(output)),
                "origin": ("byte copy of the installed native station texture"
                           if identity.endswith("01") else "source fixtures.py alternate ID, v1-M texture")})
    (evidence / "manifest.json").write_text(json.dumps(manifest, indent=2) + "\n")

    world = ET.parse(installation / "worlds/janus.sdf")
    root = world.find("world")
    station = root.find("include[uri='model://janus_dock']")
    ET.SubElement(station, "name").text = "station_primary"
    for name, model, pose in (("station_other", "janus_distractor", "1.50 -0.44 0.31 0 0 0"),
                              ("station_duplicate", "janus_dock", "1.50 -5 0.31 0 0 0")):
        include = ET.SubElement(root, "include")
        ET.SubElement(include, "uri").text = f"model://{model}"
        ET.SubElement(include, "name").text = name
        ET.SubElement(include, "pose").text = pose
    write_xml(output / "two_station.sdf", world)

    # The replay plant retains the same dynamics. It has no renderer and no dock collision.
    physics = ET.parse(installation / "worlds/janus.sdf")
    root = physics.find("world")
    root.remove(root.find("plugin[@filename='gz-sim-sensors-system']"))
    root.remove(root.find("include[uri='model://janus_dock']"))
    write_xml(output / "physics_only.sdf", physics)


if __name__ == "__main__":
    cli = argparse.ArgumentParser(description=__doc__)
    cli.add_argument("installation", type=Path)
    cli.add_argument("output", type=Path)
    args = cli.parse_args()
    prepare(args.installation, args.output)
