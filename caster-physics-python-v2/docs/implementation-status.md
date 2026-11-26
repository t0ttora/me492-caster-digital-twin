# V2 local implementation status

Updated 13 November 2026. This is the local work record; no GitHub issue, public
website, remote branch or release was changed.

15 November 2026: created `output/figures/V2_Control_Error_Geometry.svg`,
showing axle/target poses, body/world axes, tracking-error components and terminal
bearing/distance for forward approach. Vector decomposition was checked, SVG XML
parsed, and the final browser rendering visually reviewed. No control code changed.

PDF symbol rendering revised: inline mathematical quantities are now typeset
with STIX, matching displayed equations, including subscripts, superscripts,
Greek characters and derivative marks. All three final pages reviewed at 150 dpi;
no raw underscores/delimiters or replacement characters remain in PDF text.
Evidence: `evidence/pdf-symbol-rendering-audit.json`. Controller code unchanged.

V2.3: fulfilled the clarified request for V2-only control derivations and an
explicit gain calculation. New nominal tracking and sampled-wheel pole targets
produce gains in `controller_gains`; the implementation consumes those values.
37 tests pass. The current three-page English sheet is
`output/pdf/Arche_V2_Controller_Design_EN.pdf`, with every page visually checked.
This new design does not rewrite the provenance of the earlier manual gains.
V2.3 full-matrix verification also completed: all 40 cases passed their declared
numerical and completion checks, with source/parameter/result hashes verified.

Current provenance correction: `output/pdf/Arche_Mechanical_Derivation_EN.pdf`
contains three visually reviewed pages based on the user's supplied mechanical
derivation and the corresponding physics-kernel equations. No invented numerical
data or controller-gain claims are included; contact coefficients remain symbolic.
This is a mechanical derivation, not a proof of the separately designed V2
tracking controller. No simulation/controller/parameter files changed.

Latest requested hand-copy version: `output/pdf/Arche_Calculation_Proofs_EN.pdf`.
Exactly three English pages of derivations and numerical substitutions, with
only short assumption labels; all final pages visually checked. No controller
or parameter change in this document-only revision.

V2.2: wheel roll inertias aligned with the supplied uniform-cylinder V1
calculation using separate V2 working JSONs; V1 preserved. Four new regressions
cover both nominal spin inertias, isolated-wheel acceleration and caster-radius
scaling. All 35 V2 tests pass. Current English three-page PDF was created and
all pages visually inspected. [Details and evidence](inertia-alignment.md).
The aligned-parameter full study also completed: 40/40 runs passed all declared
checks, including completion; 16/16 nominal completed in the terminal window
and 8/16 within the original one-second window. V1's 60-file inventory is intact.

User-requested concise calculation supplement: created
`output/pdf/Arche_V2_Kisa_Hesap_TR.pdf`, exactly three pages, all visually checked.
It follows the supplied handwritten V1 notation and covers only the added V2
tracking, PI and terminal-control calculations, with numerical examples. It
explicitly separates the attachment's isolated-wheel inertia (0.001125 kg m²)
from the Python scalar example (0.0045 kg m²). No physics/control code changed.

V2.1 correction completed locally: preserved V1 still passes all 54 tests and
its 60-file snapshot matches; V2.1 passes 31 tests. Forty synthetic runs complete
with unchanged position/yaw thresholds and a stricter planar-speed dwell check.
Completion within the original one-second window is 8/16, compared with 3/16
before; an explicit eight-second terminal observation gives 16/16 nominal and
40/40 including sensitivities. Controller correction, horizon change and proof
assumptions are separated in [the correction report](tracking-fix.md).

| V2.1 work item | State | Evidence |
| --- | --- | --- |
| Diagnose terminal lateral equilibrium and wheel P load offset | Executed | Before diagnostics, extended-old-controller experiment, failing regressions |
| Correct tracking, terminal recovery, PI with anti-windup | Implemented and verified within synthetic matrix | 31 tests, 40 completed cases; separate original-window metrics |
| Check full feedback transient at half solver step | Verified for the selected high-drag arc | Same completion time; sub-1e-8 rad angle discrepancy |
| Prepare separate Turkish derivation and hand-calculation PDF | Created and visually verified | Twenty pages, 69 equations; all pages inspected; full numeric/RK4 example and final PDF hash in delivery-v2.1-validation.json |
| Establish global full-M2 feedback stability or real-robot validity | Not established | Ideal sub-model proofs and finite synthetic experiments have narrower scope |

The original V2 implementation record follows. Its completion failures describe
the archived initial controller, not the corrected V2.1 controller.

| Work item | State | Evidence |
| --- | --- | --- |
| Read project announcement and current public repository vision | Executed | Extracted scope/hash in evidence; inspected main cc6331f596f55387c7eaa2a4e9753aef5cec42aa |
| Preserve V1 without modifying source/parameters/reference/tests | Verified | 60-file SHA-256 inventory and archive, archive hashes checked |
| Rerun V1 existing tests | Verified | 54 tests passed, evidence/v1-tests.xml |
| Rerun recorded MATLAB comparison and physical checks | Verified within recorded/synthetic scope | 3 complete series, 12 endpoints, 78 checks; 258 original files unchanged |
| Add separate V2 experiment layer | Implemented and executed | twin.py, run_study.py, separate output tree; original M2 kernel reused |
| Four maneuvers × two profiles × two synthetic surfaces | Executed | 16 nominal runs in the 40-run study |
| Caster trail/radius/damping partial sensitivity | Executed | 24 additional runs, same study |
| Independent mechanics identities, sampled odometry and preservation/replay tests | Verified within numerical scope | 24 V2 tests passed, including 40 random states |
| Recording evaluator and strict SI/time/frame/provenance contract | Implemented; synthetic CLI verified | evaluate_recording.py; synthetic replay has zero pose discrepancy |
| Inspect representative rendered figure | Executed | Reference-surface smooth arc plot inspected; no layout defect observed |
| Check result/source hashes and archive inventory | Verified | evidence/delivery-validation.json, all_passed true |
| Claim improved tracking/controller optimization | Not established | Many completion failures; spatial error not uniformly improved |
| Check user's handwritten derivation line by line | Pending input | Scope announcement supplied, handwritten equations not supplied |
| Confirm actual platform dimensions, motor interface and measured parameters | Pending physical evidence | No real platform data supplied in this task |
| Fit common drive response and small identifiable contact parameter set | Pending development recordings | Do not tune on confirmation runs |
| Validate against independent physical pose on two approved surfaces | Pending robot/log access | Synthetic evidence cannot satisfy this item |
| Choose/freeze one improved profile and perform confirmation trials | Pending development/confirmation protocol | Current quintic candidate remains provisional |
| 3D engine integration, material model, ramps and resolved shimmy | Not implemented in this V2 increment | Not inferred from planar numerical tests |
| Public publication / robot actuation | Not performed | Local files and offline simulation only |

Next practical gate: obtain the actual platform/interface inventory and one
quality-checked pilot recording. Existing local MATLAB/CAD and prior worktree
changes remain in place. V2 adds no dependency, simulator runtime or robot access.
## Formula references (15 November 2026)

The current three-page English controller calculation PDF uses short arrow callouts from formulas to
source locators. The equation-level mapping and short oral-defence answers are
in [formula-sources.md](formula-sources.md). References distinguish the original
Kanayama law from V2's adaptation, textbook methods from V2-specific algebra,
and selected poles/time constants from measured parameters. All three rendered
pages were visually inspected; page count, arrow labels and raw math checks
passed. No controller code changed or simulation was rerun for this document-only
revision. The current PDF hash is recorded in the two PDF audit files.
