#!/usr/bin/env bash
set -euo pipefail
codec="${JANUS_CODEC_DIR:-$HOME/janus-mission-codec}"
mkdir -p -- "$codec"
cd -- "$codec"
apt-get download python3-zxing-cpp=2.2.1-3 libzxing3=2.2.1-3
sha256sum -c <<'HASH'
2868d46914a0b244ac6428afb24d5849f9730834bb919f419bce1cbe07b3f4ee  libzxing3_2.2.1-3_arm64.deb
5d6e7dd1b0506cc348dfee6773b7e6c0211c1a4fd71760e93faaaa0404861531  python3-zxing-cpp_2.2.1-3_arm64.deb
HASH
for archive in ./*.deb; do
    dpkg-deb -x "$archive" extracted
done
