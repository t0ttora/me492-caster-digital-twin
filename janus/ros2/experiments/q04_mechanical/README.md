# Q04 staged mechanical alignment supplement

This candidate keeps the retained Q04 launch fixture and Gazebo world byte-for-byte. The first optical-only run remains the comparison baseline. The exact launch used Ubuntu 24.04 arm64, ROS 2 Jazzy, Gazebo Harmonic and the zbar backend for the single rendered marker. The ground-truth topic is consumed by a dedicated measurement callback. Driving uses wheel odometry and the camera.

The station center uses seven distinct rendered four-corner observations while the chassis is stopped. Each candidate uses the nominal camera offset and marker extent to estimate center from optical edge heights. The heading prior is the declared synthetic marker yaw relative to the declared spawn yaw. The measured marker-yaw estimates are retained for inspection. The controller enters three odometry-guided staging phases, each inhibited by invalid or expired QR data. Staging travel has its own 0.10 m/s limit. The final optical approach uses the shared fresh-frame inner band, dwell and outer band. This staging segment is distinct from the image-centered final approach.

The independent verifier reads only the retained inputs, emitted requests, QR observations and Gazebo world poses. The terminal gates retain the existing 245–265 mm optical depth range, ±2% camera-center band, 20 mm position tolerance, 0.05 rad yaw tolerance and speed limits of 0.02 m/s and 0.05 rad/s for at least 250 ms.

## Reproduction

Use the built JANUS Jazzy overlay inside the existing Ubuntu guest. Prepare an input directory with the exact original Q04 `fixture.json` and `world.sdf` from the optical-only native result. This recipe uses ROS domain 87, localhost discovery, and a per-run Gazebo partition. Reserve domain 87 before launching.

```sh
bash run_guest.sh /guest/path/to/q04-inputs /guest/path/to/new-q04-result 140
python3 verify.py /guest/path/to/new-q04-result /guest/path/to/original-q04-result
python3 plot_comparison.py /guest/path/to/new-q04-result /guest/path/to/original-q04-result q04-comparison.png
```

The original result directory must include `acceptance.json` and `telemetry.jsonl` for comparison. Use the repository plotting environment with Matplotlib for the last step. A local audit can copy the new result to the host and compute the same verification.

## Retained inputs and scope

- World SHA-256 `8e6ad2a277c2601a513073b0b93a6976332a0deb24f001426f59b83789e8d71a`.
- Fixture SHA-256 `acaca38980cd43b554fb422fb455707282c5389e86565a73d3887d6a63939bbb`.
- Original optical-only acceptance SHA-256 `5f5728509b397e95746889ba7b36db6e96c519ea565d0b19bfa22a6e4dd76f69`.

The measured result applies to the unchanged synthetic Q04 fixture. Physical dock contact, charging, camera installation and station yaw calibration remain separate measurements.
