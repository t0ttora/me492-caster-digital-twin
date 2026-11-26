"""Exercise actual QR image decoding independently of controller projection."""
import argparse
from datetime import datetime, timezone
from pathlib import Path

from janus_artifacts import atomic_json, digest
from qr_camera import DOCK_ID


def decode_fixture(output):
    import cv2
    from PIL import Image
    import qrcode

    other = "JANUS-OTHER-99"

    def tile(payload):
        code = qrcode.QRCode(box_size=7, border=4)
        code.add_data(payload)
        return code.make_image(fill_color="black", back_color="white").convert("RGB")

    output.mkdir(parents=True, exist_ok=False)
    fixtures = {
        "nominal": ((DOCK_ID, 215, 80),),
        "wrong-only": ((other, 90, 72),),
        "two-stations": ((DOCK_ID, 49, 80), (other, 355, 139)),
        "ambiguous-id": ((DOCK_ID, 49, 80), (DOCK_ID, 355, 139)),
        "blank": (),
    }
    detector = cv2.QRCodeDetector()
    results = []
    for name, placements in fixtures.items():
        path = output/f"{name}.png"
        image = Image.new("RGB",(640,480),"white")
        for payload, x, y in placements:
            image.paste(tile(payload), (x,y))
        image.save(path)
        ok, payloads, positions, _ = detector.detectAndDecodeMulti(cv2.imread(str(path)))
        expected = sorted(payload for payload, _, _ in placements)
        payloads = list(payloads) if ok else []
        candidates = [{"id": payload, "corners_px": quad.tolist(), "center_px": quad.mean(axis=0).tolist()}
                      for payload, quad in zip(payloads, positions if ok else [])]
        matching = sum(payload==DOCK_ID for payload in payloads)
        passed = sorted(payloads)==expected and ((matching==1) == (expected.count(DOCK_ID)==1))
        results.append({"fixture":name, "image":path.name, "sha256":digest(path), "expected_ids":expected,
                        "candidates":candidates, "selection": DOCK_ID if matching==1 else None, "passed":passed})
    report = {"schema_version":"janus-python-1","evidence_scope":"synthetic_image_decode",
              "created_utc":datetime.now(timezone.utc).isoformat(), "opencv_version":cv2.__version__,
              "scope":"Static generated PNGs decoded by OpenCV. Timing, optics and closed-loop camera response are outside this experiment.",
              "checks_passed":all(result["passed"] for result in results),"cases":results}
    atomic_json(output/"decode_report.json",report)
    print(output/"decode_report.json")
    return report["checks_passed"]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output-dir",type=Path,required=True)
    args = parser.parse_args()
    if not decode_fixture(args.output_dir):
        raise SystemExit(1)


if __name__=="__main__":
    main()
