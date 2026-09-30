# Learning Model Work Package

Secondary objective | Motion prediction within 28 hours

## Prediction task

RQ3, a secondary objective, reuses RQ1/RQ2 recordings to test predictive value beyond simple baselines. Predict yaw tracking-error magnitude 0.5 seconds ahead from roughly 0.5 seconds of causal commands, speed, yaw rate, acceleration and recent error. Include caster angle/current only if reliable. Freeze the horizon and channels on development data. This is a motion-disturbance proxy; caster attribution requires caster measurements.

## Smallest defensible model

Compare persistence, a transparent numerical/rule baseline and one regularized regression. A compact nonlinear model requires a specific development gap and room in the 28-hour budget. Check any existing Arche model for task fit and permission before reuse. A language model is not the default for numerical telemetry. Foundation-model training, RL and a full robot intelligence stack are outside scope.

| Step | Hours | Output |
| --- | --- | --- |
| Task, labels and run split | 6 | Causal feature/target specification and provenance. |
| Data checks and simple baselines | 8 | Persistence/rule results and leakage checks. |
| Training and development comparison | 8 | One model, preprocessing and locked settings. |
| Independent evaluation and write-up | 6 | Test metrics, model card and benefit/limitation decision. |

## Evaluation rules

Split by entire run and preferably by recording session. Fit scaling and feature selection on training data only. The 24 calibration runs provide development data; blocked validation reserves whole runs. The 48 confirmation runs remain untouched until 7 December. Overlapping windows are correlated and do not create independent robot trials. Synthetic data remain labelled and cannot constitute real-world validation.

Report prediction MAE against persistence, per-run spread and performance across the two surfaces. If a warning threshold is added, choose it on development data and report missed events and false warnings per minute on the holdout. Rare events or inadequate labels trigger G5’s feasibility outcome. Report inference latency on the actual target computer; laptop timing alone cannot establish robot deployment readiness.

The model can remain offline. Deployment requires runtime-available channels: external reference error can label evaluation but cannot become an input without that reference online. Deployment needs Arche authorization and held-out benefit. A second platform or broader events belong to later work supported by evidence and permission.
