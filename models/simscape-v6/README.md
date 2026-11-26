# Native Simscape caster models and alignment replay

Open the SLX files in MATLAB R2026b with Simulink, Simscape and Simscape Multibody.

| File | Configuration |
| --- | --- |
| `caster_visual_v5_cylinder_contact.slx` | Historical native contact-cylinder version. |
| `caster_visual_v6_measurements.slx` | Saved measurements model with friction disabled and a zero-degree initial caster angle. |
| `evidence_latest_alignment_replay.slx` | Reconstructed 3 s experiment with export sinks, a +30° relative caster angle, smooth stick-slip at the three contacts, and 0.01 N m per drive wheel. |

To rerun the reconstructed case, open `evidence_latest_alignment_replay.slx` and press **Run**. The model uses fixed-step ode4 at 25 μs. The export sinks sample at 100 μs and save timeseries in the simulation output. The archived files below describe the run recorded on 16 November 2026.

- [Full replay CSV](evidence/full_replay.csv) and [run manifest](evidence/replay_manifest_enriched.json)
- [Replay and original retained interval comparison](evidence/v6_alignment_replay_and_original_tail.png)
- [V5 solver step-size comparison](evidence/v5_step_refinement.png)
- [Public progress and validation limits](../../docs/progress/2026-11-27-simulation-and-source.md)

The replay reproduces the 2.750025–3 s retained interval. Earlier samples in the full CSV come from the reconstruction. Surface coefficients and geometry are synthetic.
