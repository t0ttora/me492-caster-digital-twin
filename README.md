# Arche Robotics · Caster-aware digital twin

**A mobile-robot study of passive caster dynamics, motion planning and simulation-to-reality agreement.**

Oluş Emre Demir · Mechanical Engineering, Yeditepe University · ME492 · Fall 2026

This project investigates how passive caster wheels affect mobile-robot motion and whether accounting for their dynamics can improve simulation fidelity and trajectory tracking. The intended outcome is a calibrated digital twin, a repeatable comparison of conventional and smoother motion profiles, and engineering conclusions supported by measured evidence.

The study is being prepared around an Arche Robotics platform. Its actual configuration, available telemetry and permitted experiments must be confirmed with the company. This repository is the public project record: research methods, execution plans, decisions, progress and permitted reports. It currently contains documentation; simulation software and experimental results have not yet been published.

**Follow the project:** [Public research journal](https://t0ttora.github.io/me492-caster-digital-twin/) — progress, vision, tasks, roadmap, decisions and reports.

## The engineering problem

An ideal differential-drive model describes motion through the driven wheels, but a real platform also interacts with passive casters and the floor. Caster reorientation, rolling resistance and contact effects can contribute to deviations during turns, reversals and speed changes. The study will test when those effects matter and whether a more detailed model or a smoother command profile provides a useful improvement.

A digital twin is useful here only if its predictions can be compared with the real robot under matched conditions. The project therefore connects mechanical modeling, simulation, calibration and independent evaluation rather than treating a working simulation as proof of physical accuracy.

## Objectives and research questions

| Objective | Question | Intended evidence |
| --- | --- | --- |
| **Caster-aware digital twin** | RQ1: How much does an explicit caster model improve reproduction of measured motion over an ideal drive model? | Ideal and caster-aware predictions compared against the same measured commands and reference motion. |
| **Smoother motion profiles** | RQ2: Can a smoother speed/turn profile reduce tracking and yaw error without an unacceptable completion-time penalty? | Repeated baseline/improved-profile trials; tracking error, yaw error and completion time. |
| **Short-horizon prediction — secondary** | RQ3: Can recent telemetry predict yaw tracking error better than persistence and simple numerical/rule baselines? | One compact predictor evaluated on independent runs, with its limitations reported. |

The mechanical model, digital twin and motion-profile comparison are the primary deliverables. The prediction study reuses their recordings and remains secondary; it does not replace the physical validation work.

## Method and scope

1. **Identify the platform:** confirm drive layout, wheel/caster geometry, mass properties, command interfaces, telemetry and access permissions.
2. **Build comparable models:** establish an ideal drive baseline, then add a bounded caster/contact model and replay matched commands.
3. **Pilot and calibrate:** check logging, timing, coordinate frames and an independent pose reference; fit a small identifiable parameter set.
4. **Select a motion profile:** compare candidate speed/turn profiles on development evidence and lock one finalist before confirmation trials.
5. **Evaluate and report:** compare the models and profiles on independent recordings, quantify uncertainty and retain negative findings.

The planned study covers **one robot configuration, two approved surfaces, four maneuver families and one improved profile**. The requested experiment matrix contains 24 calibration/development runs and 48 confirmation runs. Three repeats per cell are a practical starting minimum, not a statistical power guarantee. Data splits are by whole run/session, and the confirmation set is held back from tuning.

The actual platform determines the implementation environment. Geometry, frames, parameters, command histories and run identifiers must remain traceable across simulation and recordings. See the [experiment protocol](docs/research/experiment-protocol.md) and [secondary prediction task](docs/research/learning-task.md) for the detailed evaluation rules.

## Intended deliverables

- A documented platform model and calibrated caster-aware digital twin, compared with an ideal baseline.
- A repeatable experiment and simulation procedure with run provenance and explicit metrics.
- A measured baseline/improved-profile comparison across the approved surfaces.
- A compact prediction benchmark or a documented feasibility outcome if the data are inadequate.
- A final engineering report, traceable figures and a reproducible handoff within publication permissions.

Physical validation depends on suitable robot access, logs and reference measurements. If those are unavailable, an advisor-reviewed scope reduction must be recorded, and a simulation-only outcome must be described as such. A broader robot intelligence stack, foundation-model training, reinforcement learning and additional platforms are outside the semester scope.

## Repository guide

```text
.
├── README.md               Project purpose, objectives and scope
├── CONTRIBUTING.md         How to maintain the public record
├── docs/
│   ├── planning/           Timetable, tasks, decision gates and decisions
│   ├── research/           Experiment protocol, prediction task and notebook
│   ├── access/             Company access checklist and permission register
│   ├── progress/           Dated work and evidence records
│   └── sources.md          References and baseline provenance
├── tests/                  Record consistency and website checks
├── reports/                Published PDFs and report index
├── site/                   Research website design and brand assets
├── scripts/                Static website builder and checks
└── .github/                Issue templates and Pages publication workflow
```

| Looking for | Start here |
| --- | --- |
| Advisor dashboard | [Build, preview and publication guide](site/README.md) |
| Reports and advisor handoff | [Report library](reports/README.md) |
| Semester roadmap | [Timetable](docs/planning/timetable.md) and [task / gate index](docs/planning/task-index.md) |
| Research methods | [Experiment protocol](docs/research/experiment-protocol.md), [prediction task](docs/research/learning-task.md) and [research notebook](docs/research/research-log.md) |
| Scope decisions | [Decision gates](docs/planning/decision-gates.md) and [decision log](docs/planning/decisions.md) |
| Company access | [Access register](docs/access/access-register.md) and [visit checklist](docs/access/access-inventory.md) |
| Actual work and evidence | [Progress history](docs/progress/README.md) and [GitHub issues](https://github.com/t0ttora/me492-caster-digital-twin/issues) |
| References | [Sources](docs/sources.md) |

## Project status and participation

The published baseline is a preparation document. As of **30 September 2026**, company access is pending and this repository contains no verified physical experiment results or trained model. Dated updates belong in the [progress history](docs/progress/README.md); live task status belongs in [issues](https://github.com/t0ttora/me492-caster-digital-twin/issues) and [milestones](https://github.com/t0ttora/me492-caster-digital-twin/milestones).

The semester plan allocates 252 working hours plus 32 contingency hours. Academic deadlines and weekly acceptance evidence are maintained in the [timetable](docs/planning/timetable.md). Completing an issue records task progress; it does not establish research validity or company approval.

To add work, link the relevant research question, record the actual evidence and distinguish **prepared**, **executed** and **independently verified** states. See [CONTRIBUTING.md](CONTRIBUTING.md). Company CAD, raw telemetry, model weights and restricted reports may be published only with explicit permission; the public record contains only permitted material.
