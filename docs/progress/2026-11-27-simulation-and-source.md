# MATLAB, Simscape and Python simulation update

Published 27 November 2026. This entry covers the October modelling work and November simulation records.

## MATLAB and Simscape

| Model | Work and evidence |
| --- | --- |
| M0 kinematic reference | Six synthetic command cases. The Windows verifier passed 32 checks. An independent review of the archived series passed 56 geometry and kinematics checks. |
| M2 planar model | Independent drive torques, a passive caster, coupled inertia and explicit support loads. MATLAB R2026b executed 12 scenarios. The v1 model passed 60 numerical and 13 workflow checks. The review package includes three result series and five native figures. |
| Native Simscape V5 | Contact-cylinder refinement and solver step-size comparisons. The independent audit passed 19 of 20 checks. The swivel-rate difference between 50 and 25 microsecond steps was 0.00548 rad/s against a 0.0001 rad/s criterion. |
| Native Simscape V6 | The saved measurement model and the reconstructed +30° alignment replay are available. The 3 s replay used a 25 microsecond solver step and recorded 30,001 samples. The original log retains 2.750025–3 s. Comparison over that interval gave a maximum caster-angle difference of about 1.12 × 10⁻⁶ degrees. |

[MATLAB M0 source](https://github.com/t0ttora/me492-caster-digital-twin/tree/main/models/caster-m0) · [MATLAB M2 v1 source and verification](https://github.com/t0ttora/me492-caster-digital-twin/tree/main/models/caster-m2-v1) · [Simscape models and replay data](https://github.com/t0ttora/me492-caster-digital-twin/tree/main/models/simscape-v6)

## Python models and controller

Python V1 ports the planar M2 physics and checks against archived MATLAB runs. V2.3 adds time-parameterized reference profiles, tracking, terminal pose recovery and wheel-speed PI control with gains derived from documented design targets. The synthetic study covers four manoeuvres and two profiles on two parameterized surfaces, plus a caster sensitivity sweep. The terminal lab runs either model and opens the recorded metrics and plots.

[Python V1](https://github.com/t0ttora/me492-caster-digital-twin/tree/main/caster-physics-python) · [Python V2.3](https://github.com/t0ttora/me492-caster-digital-twin/tree/main/caster-physics-python-v2) · [Terminal lab](https://github.com/t0ttora/me492-caster-digital-twin/tree/main/arche-tui) · [Controller calculations](../../reports/Arche_V2_Controller_Design_EN.pdf)

[Academic progress report](../../reports/Arche_Academic_Report_Typeset_EN_2026-11-16.pdf) · [V2.3 synthetic study results](../research/python-v2-3-synthetic-study.md)

## Remaining validation

Published simulations use provisional inputs. Physical calibration requires platform parameters and synchronized command, wheel and independent pose measurements. The Simscape swivel-rate refinement and the native −30° mirror case remain open. The original early portion of the +30° log is unavailable. There are zero measured runs in the public run inventory.
