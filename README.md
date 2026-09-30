# ME492 | Caster-aware digital twin

**Oluş Emre Demir · Yeditepe University · Individual project · Fall 2026**

Public execution and research record for the Arche Robotics mobile-robot study. The main engineering objective is the caster-aware twin and motion-profile comparison. Compact motion-error prediction is a secondary objective using the same recordings.

**Start here:** [Latest 11-page execution baseline](reports/ME492_Final_Roadmap_and_Arche_Visit_EN.pdf) · [2-page timetable](reports/ME492_Timetable_EN.pdf) · [Weekly progress](docs/progress/README.md)

## Current snapshot - 30 September 2026

| Item | Actual state |
| --- | --- |
| Execution baseline | Prepared; revised English PDF published |
| Work under way | W01 scope/preparation; implementation and physical evidence remain to be completed |
| Arche visit | Requested for 6/7 Oct; unconfirmed, fallback 13/14 Oct |
| CAD / robot / telemetry / AI access | Pending company decisions |
| Physical experiments / trained model | No results or completion claimed in this tracker |
| Next decision | G1 platform and access, 16 Oct |
| Capacity | 252 planned hours + 32 contingency; reserve is not feature development |

This snapshot is dated. The issue lists below show the current work state; progress updates record what was actually executed and verified. Publishing a plan does not establish physical validation.

## Task management

[Open the full task and gate index](docs/task-index.md).

[In progress](https://github.com/t0ttora/me492-caster-digital-twin/issues?q=is%3Aissue%20is%3Aopen%20label%3Astatus%3Ain-progress) · [Planned](https://github.com/t0ttora/me492-caster-digital-twin/issues?q=is%3Aissue%20is%3Aopen%20label%3Astatus%3Aplanned) · [Blocked](https://github.com/t0ttora/me492-caster-digital-twin/issues?q=is%3Aissue%20is%3Aopen%20label%3Astatus%3Ablocked) · [Verified / closed](https://github.com/t0ttora/me492-caster-digital-twin/issues?q=is%3Aissue%20is%3Aclosed%20label%3Astatus%3Averified) · [All weekly tasks](https://github.com/t0ttora/me492-caster-digital-twin/issues?q=is%3Aissue%20label%3Atype%3Aweekly) · [Decision gates](https://github.com/t0ttora/me492-caster-digital-twin/issues?q=is%3Aissue%20label%3Atype%3Agate) · [Milestone dates](https://github.com/t0ttora/me492-caster-digital-twin/milestones)

Each of the 16 weekly issues has an hour budget, a concrete output, acceptance evidence and a progress checklist. The six gate issues record separate evidence-dependent decisions. Issue closure and milestone percentages are not research-validity scores. [How to update the record](CONTRIBUTING.md).

## Research questions

1. **RQ1:** To what extent does an explicit caster-aware model improve the reproduction of measured mobile-robot motion compared with an ideal drive model?
2. **RQ2:** Can a smoother speed/turn profile reduce tracking/yaw error without an unacceptable increase in completion time?
3. **RQ3 - secondary:** Can recent telemetry predict short-horizon yaw tracking error better than simple persistence/rule baselines?

One robot configuration, two approved surfaces, four maneuver families, one improved profile and one compact learned model. Requested experiments: 24 calibration/development runs + 48 confirmation runs. Three repeats per cell are a practical minimum, not a statistical power guarantee. Whole-run splits and model/profile locking protect the final holdout.

## Evidence and research library

| Record | What the advisor can inspect |
| --- | --- |
| [Report library](docs/reports.md) | Current PDFs and future dated research outputs |
| [Semester timetable](docs/timetable.md) | Weekly outputs, hours and acceptance evidence |
| [Research notebook](docs/research-log.md) | Questions, methods, findings, uncertainty and limitations |
| [Experiment protocol](docs/experiment-protocol.md) | Reference measurements, replication and control rules |
| [Secondary prediction task](docs/learning-task.md) | Baselines, causal channels, splits and evaluation |
| [Decision gates](docs/decision-gates.md) | Evidence, deadlines and reduced-scope paths |
| [Decision log](docs/decisions.md) | Recorded choices and the authority/evidence behind them |
| [Access register](docs/access-register.md) | Pending versus confirmed company support |
| [Visit checklist](docs/access-inventory.md) | Questions to resolve with Arche |
| [Run inventory](docs/run-inventory.csv) | Run/session IDs and split provenance; currently empty |
| [Sources](docs/sources.md) | References underlying the baseline |

## Fixed academic dates

GDS: **6 Nov 2026, 12:00-15:00** · Draft: **4 Dec** · Internal final: **23 Dec** · Package: **25 Dec** · Submission-ready: **8 Jan 2027** · Final report: **13 Jan, 23:59** · Defense session: **14 Jan, 09:30-12:00**. All times Europe/Istanbul. [Calendar source](https://yeditepe-me492.github.io/2026-Fall/).

## Weekly visibility

Monday: choose one demonstrable output. Tuesday/Wednesday: use confirmed company slots or modeling time. Friday: publish actual work, evidence, hours, blocker, next output and decision request. Public pages are readable without a GitHub account; commenting/editing requires an account. Updates are manual and nothing is automatically sent to the advisor.

This is the public progress subset. Company CAD, raw telemetry, model weights and restricted reports are not included; future material is published only within explicit permission.
