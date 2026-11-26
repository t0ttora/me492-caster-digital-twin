# V2.2 wheel inertia alignment

13 November 2026. The active parameter set is `parameters/` in this V2 folder.
The V1 Python and MATLAB parameter/reference files remain frozen historical
inputs, and their numerical agreement is not evidence for the corrected values.

## Finding

The handwritten V1 PDF, page 5, specifies 0.4 kg drive wheels of radius 0.075 m
and a 0.4 kg caster wheel of radius 0.040 m. For a uniform solid cylinder about
its own spin axis, integrating concentric rings gives `J = m*r^2/2`.

| Quantity | Frozen Python/MATLAB | Aligned V2.2 | Meaning |
| --- | ---: | ---: | --- |
| Drive roll inertia, each wheel | 0.0045 | 0.001125 kg m² | 0.5 × 0.4 × 0.075² |
| Caster roll inertia | 0.00045 | 0.000320 kg m² | 0.5 × 0.4 × 0.040² |

The drive value 0.0045 was inherited from the provisional MATLAB M2 parameter
file. No physical derivation or identified motor/gear contribution was found in
the inspected sources. It is four times the uniform-cylinder value. Furthermore,
any 0.4 kg wheel whose mass lies within radius 0.075 m satisfies `J <= m*r^2 =
0.00225 kg m²`; thus 0.0045 cannot describe that wheel alone. An effective
drivetrain inertia could be larger, but would need separate measured/derived
rotor and gear data. None is added here. The exact cause of the original number
is unknown; radius/diameter confusion is not established as its cause.

The caster value 0.00045 is possible for another radial mass distribution,
but does not match the uniform-cylinder assumption explicitly used in the sheet.
The aligned value is correct for the declared ideal geometry, not a measurement
of the real platform. Body/caster yaw inertias remain distinct provisional
parameters; they were not changed or represented as CAD-calibrated values.

Reference derivation: [OpenStax, University Physics Vol.1, section 10.5](https://openstax.org/books/university-physics-volume-1/pages/10-5-calculating-moments-of-inertia).

## Implementation and checks

- V2 now loads its own SI JSON working parameters, using the unchanged V1 physics
  kernel and validator. The declared wheel masses and geometric assumption are
  recorded as provenance in the V2 dynamics JSON.
- Caster radius sweeps retain wheel mass and scale roll inertia with radius
  squared. They no longer change radius while silently retaining nominal roll
  inertia. Other mass/yaw properties still need a real component model.
- The PI scalar load test now uses the active drive inertia. Gains and limits
  are unchanged. With J=0.001125 and 10 ms sampling, the unsaturated scalar PI
  poles are approximately -0.351841 and 0.947397, both inside the unit circle.
- Four regressions failed before the correction. The full V2.2 suite then passed
  35 tests. The isolated-wheel test explicitly disables drive traction, rolling
  resistance and viscous drag: 0.01 N m / 0.001125 = 8.888889 rad/s², yielding
  1.777778 rad/s after 0.2 seconds from rest. The body remains stationary.
- Current full study: `outputs/inertia-aligned-v2.2-2026-11-13/`. It records both
  source and working-parameter hashes. Previous V2.0/V2.1 studies are historical.
- Completed verification with the aligned values: all 35 tests and all 40 study
  runs passed their declared checks; 16/16 nominal runs completed with the 8 s
  terminal observation, and 8/16 within the original 1 s window. Energy/passivity
  and torque-limit checks passed. Selected half-step feedback refinement also
  passed, with maximum caster-angle discrepancy approximately 1.05e-6 rad.
  Final source/parameter/CSV hashes and V1 preservation were verified in
  `evidence/inertia-alignment-audit.json`.
- Current three-page English calculation: `output/pdf/Arche_V2_Calculations_EN.pdf`.
  Earlier Turkish/long reports retain their versioned historical values.

No archived MATLAB outputs were rewritten to make a comparison pass. No hardware
measurement or motor/gear inertia identification was performed.
