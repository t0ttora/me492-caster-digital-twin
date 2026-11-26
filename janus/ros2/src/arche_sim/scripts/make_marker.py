#!/usr/bin/env python3
"""Bake the synthetic station texture with a four-module quiet zone."""
import json
from pathlib import Path
import sys

import qrcode
from qrcode.constants import ERROR_CORRECT_M


def write_marker(config_path, output_path):
    marker = json.loads(Path(config_path).read_text())["marker"]
    code = qrcode.QRCode(version=1, error_correction=ERROR_CORRECT_M,
                         box_size=16, border=marker["border_modules"])
    code.add_data(marker["id"])
    code.make(fit=False)
    if code.modules_count != marker["modules"]:
        raise ValueError("QR module count differs from marker calibration")
    code.make_image(fill_color="black", back_color="white").convert("RGB").save(output_path)


if __name__ == "__main__":
    write_marker(*sys.argv[1:])
