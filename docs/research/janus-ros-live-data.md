# ROS 2 and Gazebo run data

These three runs used Ubuntu 24.04 arm64, ROS 2 Jazzy and Gazebo Harmonic. All commands and measurements come from the simulator.

| Case | Accepted run | What the files show |
| --- | --- | --- |
| M02 | native-isolated-v2 | The stop band was reached 63 ms after braking. [Acceptance JSON](../../reports/janus/ros2/live/M02-acceptance.json) · [Telemetry CSV](../../reports/janus/ros2/live/M02-telemetry.csv) |
| Q08 | native-q08-terminal-v2 | The Gazebo image decoder read 47 wrong-station observations. The controller aborted after 3.200 s of simulation time. It issued no forward requests or success events. [Acceptance JSON](../../reports/janus/ros2/live/Q08-acceptance.json) · [Telemetry CSV](../../reports/janus/ros2/live/Q08-telemetry.csv) · [Gazebo camera frame](../../reports/janus/ros2/live/Q08-gazebo-camera.png) |
| O05 | native-o05-recovery-v2 | The guard rejected 18 unauthorized requests while fresh odometry returned. Authorization and a new command preceded measured motion at 2.340 s. [Acceptance JSON](../../reports/janus/ros2/live/O05-acceptance.json) · [Telemetry CSV](../../reports/janus/ros2/live/O05-telemetry.csv) |

The CSVs export selected fields from the recorded telemetry in capture order. Each row has a source, such as the simulation clock, world pose, odometry, joint states, a controller request, a guard output or a QR observation. A blank field means that source did not supply it. Times use seconds, distances use meters, linear velocity uses meters per second and angular values use radians or radians per second. The case checks in each JSON also retain summary counts and configured limits. Q08 also reports the detector latency in wall-clock milliseconds.

[ROS 2 source and runbook](https://github.com/t0ttora/me492-caster-digital-twin/tree/main/janus/ros2) · [JANUS experiment record](../progress/2026-11-27-janus-experiments.md)
