# Research notebook

## 27 November 2026 · JANUS simulation results

- **RQ1 · Caster-aware dynamics.** Matched-input Python and Simscape replay and native caster reversal checks provide simulation evidence. Platform geometry and synchronized, independent pose measurements are still needed for physical calibration and model error. [JANUS experiment record](../progress/2026-11-27-janus-experiments.md)
- **RQ2 · Motion profiles.** V2.3 compares baseline and smoother profiles on synthetic surface presets. The planned robot comparison still needs approved physical surfaces, an agreed profile and an independent confirmation set. [V2.3 synthetic study](python-v2-3-synthetic-study.md)
- **RQ3 · Short-horizon prediction.** The protocol specifies yaw-error prediction from telemetry with run-level splits and simple baselines. ViNT offline inference and a Gazebo replay interface check are documented separately. The telemetry predictor and held-out comparison remain open. [Learning work package](learning-task.md) · [JANUS experiment record](../progress/2026-11-27-janus-experiments.md)

The [qualification matrix](../../reports/janus/qualification-matrix.tsv) records the 40 simulation cases. The [weekly evidence review](../progress/2026-11-27-weekly-review.md) links the semester schedule and the outstanding approvals.

## 30 September 2026 - scope and protocol revision

**Question:** how can an individual semester study answer specific mechanical and predictive questions within uncertain company access?

**Prepared result:** RQ1 compares ideal/caster-aware predictions under matched measured commands; RQ2 compares conventional/smoother profiles with an agreed time penalty; RQ3 tests compact prediction from the same recordings against persistence/rule baselines. These are planned questions, not findings.

**Method:** 24 calibration/development runs and 48 confirmation runs are requested. Split by whole run/session; seal confirmation data until model/profile/metric locking. Three repeats per cell are an access-constrained engineering minimum and do not establish statistical power.

**Evidence:** [execution baseline](../../reports/ME492_Final_Roadmap_and_Arche_Visit_EN.pdf), [experiment protocol](experiment-protocol.md), [learning task](learning-task.md).

**Open uncertainties:** actual configuration, logging quality, independent reference and available robot time. Prior architecture choices remain hypotheses until the platform visit.

For subsequent entries record: date, RQ, hypothesis, source/experiment, run and configuration IDs, result, uncertainty, interpretation, limitations and next decision. Keep unsuccessful experiments and contradictions. Do not replace missing measurements with estimated results.
