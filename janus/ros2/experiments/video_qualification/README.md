# JANUS Q09–Q12 camera qualification

This experiment takes ROS RGB8 images from the Gazebo camera. It preserves their original bytes and camera stamps, generates versioned pixel disturbances, and replays the resulting RGB8 messages into the source camera observer. A source centering gate issues low-speed requests through the source ROS command guard. A separate Harmonic physics server measures body motion at the commanded stop. The camera is static in the retained scenes and the replay drive plant has no dock contact body.

## Inputs and clocks

- Native capture uses Ubuntu arm64, Jazzy, Harmonic and the production camera bridge. The capture script checks that each PNG decodes back to the exact source RGB8 bytes. The original prepatch PNG, some captures that failed decoding, and earlier replay attempts remain in the evidence lane.
- Station IDs are `JANUS-DOCK-01` and `JANUS-DOCK-99`, using the installed target asset and alternate ID from the old source fixture. Both runtime texture PNGs have verified detached Ed25519 signatures from a local qualification key. The decoder reads plain station ID text.
- The nominal and mixed/duplicate/near-edge sensors use real native captures. Glare, reflection, exposure, low light, blur, occlusion and digital crop perturbations are explicit image-domain transforms on those captures. The two remaining geometric corners in Q12 come from the clean-reference annotation or the native camera intrinsics and known station placement.
- A replay stamp belongs to the replay world clock. It joins every generated capture to actual observer arrival/publication and guard receipt. Original capture stamps are separate. Wall decode and request-to-guard delays are Linux monotonic observations.
- The optional direct delivery mode invokes the unchanged production observer image callback serially after its clock reaches the scheduled stamp. Observer publications, command requests, guard outputs, simulated ground-truth pose and odometry still cross ROS. A configured Gazebo real-time factor cap slows wall throughput while retaining the simulation clock's 0.20 s freshness and the 1 ms physics step. DDS image delivery and wall-realtime qualification require their own result.
- Pixel centering, rejected QR, body settling, mechanical alignment, contact and charging are separate observables. The shadow run of the source docking state machine uses the replay observations and measured speed with mechanical/contact/BMS inputs absent.

## Reproduction

In the Linux guest place the source `arche_experiments` package in `source/`, copies of its `qr_observer` and `command_guard` executables in `source-nodes/`, and these runners in `scripts/`. The runtime installation of the original source Gazebo models and ROS packages is expected in `~/janus-build/install`. Own the domain and partition in the lane coordination file before launching.

```bash
bash scripts/native_capture.sh "$guest_workdir"
```

Copy the resulting `capture/`, `scene/` and `native-logs/` into `originals/native/` with the original prepatch packet in `originals/prepatch/`. Set `PYTHONPATH` to the pinned `janus/ros2/src/arche_experiments` directory for fixture generation and offline decode. Transfer the lossless PNG fixtures and index to the guest working directory.
The asset check uses OpenSSL 3. Set `--openssl` to its executable when the host defaults to LibreSSL.

```bash
python scripts/fixtures_video.py originals results/fixtures
python scripts/scan_fixtures.py results/fixtures results/offline-scan.json --originals originals
JANUS_VIDEO_REALTIME_FACTOR=0.15 JANUS_VIDEO_DIRECT_CALLBACK=1 \
  bash scripts/run_live_replay.sh "$guest_workdir" replay-e --period 0.10
```

For analysis set `PYTHONPATH` to the pinned `caster-physics-python-v2` directory. The source tests and normal JANUS CLI keep their existing tiers. A fresh run should use a fresh result directory and record its own outcome.

```bash
python scripts/analyze_replay.py results/replay-e/recording results/fixtures/manifest.json \
  results/offline-scan.json results/qualification
python scripts/render_evidence.py results/fixtures results/qualification results/replay-e/recording
python scripts/encode_replay.py results/qualification results/fixtures
python scripts/verify_package.py . results/replay-e results/qualification
python scripts/build_delivery.py . results/qualification results/replay-e
```

The `delivered-rgb.mkv` files are lossless viewing copies in replay capture order. The matching video index records original frame sequence and capture timestamps. Use the raw ROS trace to inspect intermission, delivery deadlines and output timing.
