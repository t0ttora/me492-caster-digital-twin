# V2 synthetic study

Synthetic development evidence only. No robot calibration or physical validation.

| Surface | Maneuver | Design | Profile | Position RMSE m | Path RMSE m | Yaw RMSE rad | Completion s | Jerk RMS m/s3 |
| --- | --- | --- | --- | ---: | ---: | ---: | ---: | ---: |
| reference_synthetic | straight | nominal | baseline | 0.01404 | 0.01022 | 0.00775 | 4.27 | 1.464 |
| reference_synthetic | straight | nominal | smooth | 0.00309 | 0.00100 | 0.00335 | 3.75 | 0.251 |
| reference_synthetic | arc_turn | nominal | baseline | 0.01373 | 0.01104 | 0.05202 | 5.10 | 1.045 |
| reference_synthetic | arc_turn | nominal | smooth | 0.00296 | 0.00138 | 0.01175 | 3.75 | 0.202 |
| reference_synthetic | reversal | nominal | baseline | 0.02659 | 0.00963 | 0.02922 | 6.94 | 1.683 |
| reference_synthetic | reversal | nominal | smooth | 0.00455 | 0.00286 | 0.00377 | 7.50 | 0.318 |
| reference_synthetic | s_turn | nominal | baseline | 0.01546 | 0.01162 | 0.13337 | 9.63 | 0.993 |
| reference_synthetic | s_turn | nominal | smooth | 0.00342 | 0.00105 | 0.01508 | 7.50 | 0.259 |
| high_drag_synthetic | straight | nominal | baseline | 0.01304 | 0.01015 | 0.01356 | 4.16 | 1.358 |
| high_drag_synthetic | straight | nominal | smooth | 0.00417 | 0.00260 | 0.00269 | 3.75 | 0.285 |
| high_drag_synthetic | arc_turn | nominal | baseline | 0.01222 | 0.00896 | 0.04602 | 4.52 | 1.011 |
| high_drag_synthetic | arc_turn | nominal | smooth | 0.00376 | 0.00273 | 0.01117 | 3.75 | 0.174 |
| high_drag_synthetic | reversal | nominal | baseline | 0.02507 | 0.00776 | 0.00815 | 5.04 | 1.733 |
| high_drag_synthetic | reversal | nominal | smooth | 0.00530 | 0.00290 | 0.00315 | 7.50 | 0.405 |
| high_drag_synthetic | s_turn | nominal | baseline | 0.01228 | 0.00914 | 0.07780 | 7.55 | 0.969 |
| high_drag_synthetic | s_turn | nominal | smooth | 0.00382 | 0.00221 | 0.01461 | 7.50 | 0.225 |
| reference_synthetic | arc_turn | trail_30mm | baseline | 0.01276 | 0.00877 | 0.05775 | 5.07 | 1.007 |
| reference_synthetic | arc_turn | trail_30mm | smooth | 0.00388 | 0.00237 | 0.01546 | 3.75 | 0.226 |
| reference_synthetic | arc_turn | trail_70mm | baseline | 0.01405 | 0.01176 | 0.05014 | 5.07 | 1.038 |
| reference_synthetic | arc_turn | trail_70mm | smooth | 0.00256 | 0.00087 | 0.00987 | 3.75 | 0.184 |
| reference_synthetic | arc_turn | radius_30mm | baseline | 0.01374 | 0.01106 | 0.05196 | 5.10 | 1.045 |
| reference_synthetic | arc_turn | radius_30mm | smooth | 0.00297 | 0.00138 | 0.01174 | 3.75 | 0.204 |
| reference_synthetic | arc_turn | radius_50mm | baseline | 0.01384 | 0.01119 | 0.05217 | 5.11 | 1.042 |
| reference_synthetic | arc_turn | radius_50mm | smooth | 0.00295 | 0.00138 | 0.01175 | 3.75 | 0.201 |
| reference_synthetic | arc_turn | swivel_damping_half | baseline | 0.01394 | 0.01133 | 0.05266 | 5.11 | 1.044 |
| reference_synthetic | arc_turn | swivel_damping_half | smooth | 0.00295 | 0.00138 | 0.01178 | 3.75 | 0.203 |
| reference_synthetic | arc_turn | swivel_damping_double | baseline | 0.01371 | 0.01098 | 0.05133 | 5.09 | 1.038 |
| reference_synthetic | arc_turn | swivel_damping_double | smooth | 0.00296 | 0.00137 | 0.01170 | 3.75 | 0.202 |
| high_drag_synthetic | arc_turn | trail_30mm | baseline | 0.01196 | 0.00754 | 0.05247 | 4.51 | 0.973 |
| high_drag_synthetic | arc_turn | trail_30mm | smooth | 0.00460 | 0.00358 | 0.01327 | 3.75 | 0.187 |
| high_drag_synthetic | arc_turn | trail_70mm | baseline | 0.01218 | 0.00924 | 0.04409 | 4.50 | 1.009 |
| high_drag_synthetic | arc_turn | trail_70mm | smooth | 0.00337 | 0.00228 | 0.01015 | 3.75 | 0.165 |
| high_drag_synthetic | arc_turn | radius_30mm | baseline | 0.01232 | 0.00910 | 0.04668 | 4.53 | 1.013 |
| high_drag_synthetic | arc_turn | radius_30mm | smooth | 0.00375 | 0.00272 | 0.01120 | 3.75 | 0.173 |
| high_drag_synthetic | arc_turn | radius_50mm | baseline | 0.01222 | 0.00897 | 0.04605 | 4.52 | 1.011 |
| high_drag_synthetic | arc_turn | radius_50mm | smooth | 0.00376 | 0.00274 | 0.01115 | 3.75 | 0.174 |
| high_drag_synthetic | arc_turn | swivel_damping_half | baseline | 0.01222 | 0.00901 | 0.04609 | 4.52 | 1.013 |
| high_drag_synthetic | arc_turn | swivel_damping_half | smooth | 0.00374 | 0.00272 | 0.01125 | 3.75 | 0.174 |
| high_drag_synthetic | arc_turn | swivel_damping_double | baseline | 0.01227 | 0.00898 | 0.04658 | 4.53 | 1.005 |
| high_drag_synthetic | arc_turn | swivel_damping_double | smooth | 0.00379 | 0.00275 | 0.01103 | 3.75 | 0.173 |

## Interpretation

Compare profiles within each surface/maneuver/design, including duration and failures.
Spatial error uses the nearest of 501 points on the common path. Synchronized tracking error uses each profile's own clock.
Jerk is a finite-difference estimate at 100Hz; it depends on sampling and includes the stop interval.
Motor work is signed mechanical axle work, not battery consumption.

All numerical checks passed: True
