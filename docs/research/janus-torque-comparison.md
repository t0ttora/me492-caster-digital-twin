# Equal-torque Python and Simscape comparison

The November 16 Simscape V6 alignment archive was compared with a new
Python V2.3 replay of the same two constant axle torques, 0.01 N m each.
The caster started at +30° and velocities at zero. Simscape used fixed-step
ode4 at 25 µs. Python used a 0.5 ms step and a 10 ms comparison grid.
The time series distinguishes the full reconstructed 3 s run from the
2.750025–3.0 s window retained by the original Simscape log.

| Quantity | Simscape archive | Python V2.3 |
| --- | ---: | ---: |
| World x displacement at 3 s | 68.854 mm | 3.389 mm |
| Relative caster angle at 3 s | 8.372° | 29.197° |
| Mean vertical support force in retained window | 162.84588 N | 162.846 N |

Position RMSE was 29.02 mm over the reconstructed window and 60.33 mm
within the retained original window. The full-window yaw RMSE was
0.00313 rad and swivel-rate RMSE was 0.13097 rad/s. The latter compares
different plant formulations. V5 solver step-size refinement is a distinct
test with its own swivel-rate gate.

The Simscape model uses spatial penalty contact with SmoothStickSlip.
Python models planar dynamics, regularized rolling drag and quasi-static
support. The archived pose tracks the native body frame, while Python
reports the drive-axle midpoint. Their span of motion differs even though
the settled vertical loads agree. This synthetic diagnostic motivates
matched geometry, a reference transform and measured actuator/contact
response before calibration.

[Comparison figure](../../reports/janus/archive-torque-overlay.png) ·
[Aligned CSV](../../reports/janus/archive-torque-overlay.csv) ·
[Metrics and provenance](../../reports/janus/archive-torque-metrics.json) ·
[Native V6 archive](https://github.com/t0ttora/me492-caster-digital-twin/tree/main/models/simscape-v6)

Reproduce from the repository with a Python environment containing NumPy
and Matplotlib.

```sh
python janus/compare_archive.py /path/to/new/run-directory
```
