# V2.3 controller design calculation

This is a new design calculation, not a claim that the original hand-picked V2
gains were derived before implementation. V1 mechanics and its archived files
remain unchanged. The previous V2.2 source/tests/parameters are saved in
`evidence/v2.2-before-gain-design.tar.gz`.

Current calculation sheet: `output/pdf/Arche_V2_Controller_Design_EN.pdf`, exactly
three English pages. It contains no repeated V1 mechanical derivation. All three
pages were rendered and visually reviewed.

## Selected requirements and resulting gains

Requirements are design choices, not measured data. Existing model inputs are
the 0.30 m / 2 s straight profile, 0.001125 kg m² wheel spin inertia and 0.01 s
command sample interval.

- Longitudinal time constant 0.5 s: `kx=1/tau_x=2 /s`.
- Critically damped lateral nominal poles -1,-1 /s at 0.15 m/s. Linearizing
  the tracking law gives `ey_ddot+k_heading*ey_dot+ky*v0²*ey=0`, hence
  `k_heading=2*zeta*wn=2 /s`, `ky=wn²/v0²=44.444444 /m²`.
- Sampled scalar PI poles 0.8,0.5 with integral update before torque. Matching
  trace and determinant gives `a=1-p1*p2=0.6`,
  `b=(1-p1)*(1-p2)=0.1`, then `Kp=a*J/h=0.0675`, `Ki=b*J/h²=1.125`.
- Terminal aligned-position time constant 0.5 s gives `k_rho=2 /s`.
  Bearing/final-heading time constants 1/3 s give `k_alpha=k_goal=3 /s`.

`twin.controller_gains(parameters)` performs these calculations. The tracker
uses the returned values; the study manifest records actual calculated gains
and selected specifications. Gains scale with the supplied wheel inertia/sample
interval for the chosen sampled poles. No numerical optimization or calibration
is implied.

The sheet derives the ideal tracking Lyapunov identity, nominal lateral pole
placement, exact sampled scalar wheel pole placement and terminal kinematic
convergence. These are conditional submodel results, not a global proof of the
slipping, saturated, sampled hybrid M2 closed loop.

Two new regressions failed before the implementation. The updated full test
suite passed 37 tests. Current study: `outputs/calculated-gains-v2.3-2026-11-13`;
older studies retain the gains of their original versions.

Final verification completed: all 40 newly simulated cases passed completion,
energy/passivity and torque-limit checks. Source, parameter and result hashes
match, and the frozen 60-file V1 inventory remains intact. Final audit:
`evidence/controller-gain-design-audit.json`.
