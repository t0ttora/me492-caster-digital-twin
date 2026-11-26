# M2 torque-driven differential robot with a passive caster

Prepared 7 October 2026. This is our synthetic planar model, with independent left/right wheel torques and caster ground forces feeding back into body acceleration. It is not a calibrated Arche robot or a Simscape Multibody model.

## Execution and delivery

Native Windows MATLAB R2026b executed twelve Simulink scenarios and passed **57/57 checks**, exit code 0. The captured [execution record](evidence/windows-m2-run.txt) includes the command and numerical results. The generated model is at `outputs/caster_model_m2_v0/models/Me492CasterM2.slx` in the coordinated Windows workspace. The later [M2 v1 review package](../caster-m2-v1/README.md) has now transferred to this Mac with the preserved baseline SLX, revised SLX, native figures and hash-verified evidence. This directory remains the historical source mirror.

These local sources were reconstructed from the untruncated, ordered source changes of that executed Windows turn. Geometry is copied from the shared CAD SI mirror. This Mac has not rerun MATLAB. The source package contains the model builder, runner, independent verifier, geometry and dynamics; it can recreate the SLX on a MATLAB/Simulink host.

Run from this folder in MATLAB:

```matlab
run_m2;       % Builds the model if absent, then runs twelve scenarios.
verify_m2;    % Checks the implementation and actual Simulink CSV results.
open_system(fullfile('models','Me492CasterM2.slx'));
```

`build_m2` refuses to overwrite an existing model. Save manual model edits before replay. The runner rewrites its named command/result/plot outputs; archive previous runs before a parameter study. Kernel-only checks are available with `verify_m2('kernel')`.

## Parameters and equations

`geometry_si.json` mirrors the shared wheel radii, track, caster pivot and trail in [the CAD starter](../caster-v0/README.md). It is a file mirror, not a live Onshape connection. The detailed CAD enclosure is a visual/mechanical concept; its protruding bearing bosses are not collision geometry in this planar model.

`dynamics_si.json` owns synthetic mass, COM, inertia, friction and resistance. These parameters must be replaced or identified using measured manufacturer/robot data. Nominal CAD material density and face colour do not calibrate them. The M2 solver step and command interval come from this dynamics file, overriding the starter geometry file's timing values.

The thirteen integrated states are body COM world x/y, body yaw, caster angle relative to body, seven velocities (COM x/y, yaw, swivel and three wheel rolls), motor work and dissipated energy. Absolute wheel rotation angles are not integrated; wheel roll velocities and inertias are included.

`m2_geometry` maps velocities to the drive and caster contacts. `m2_mass_terms` builds the coupled generalized mass matrix and inertial bias. `m2_contact` supplies regularized slip traction, rolling resistance, relative bearing resistance and ground bore resistance. Caster slip force and bore torque share a friction-patch budget. `m2_dynamics` solves the generalized acceleration and energy accounting. The native builder connects the contact feedback to the inertia dynamics and state integrator.

Normal loads use gravity and the instantaneous support triangle, with an explicit COM and positive-load checks. They do not include acceleration-induced load transfer. The body mass is an effective lump; drive roll inertia is separate, and the caster has separate lumped mass and roll/swivel inertia. There are no calibrated motor electrical dynamics or speed controllers.

CSV `x_m`/`y_m` refer to the drive-axle midpoint projected to the ground; the runner transforms them from the integrated COM coordinates. Yaw is positive counterclockwise; relative caster angle is measured from body forward. All physical parameters and CSV fields use SI units.

## Verified scope

Scenarios cover aligned and ±45-degree misaligned caster motion under identical torques, caster tangential-contact ablation, unequal drive torque, reversal, rest, passive coasting, force-free conservative motion, two step refinements, and reversal with swivel/bore resistance disabled. The ablation retains caster mass, vertical support and bearing resistance; it disables tangential contact effects.

Checks cover symmetry/signs, actual caster influence on trajectory, finite results, positive normal loads, friction budgets, passive dissipation, independent energy-power balance, energy/momentum conservation and step convergence. The largest integrated energy balance error was 3.80194312e-7 J against a 1e-5 J tolerance. Caster response under reversal depends on the resistance parameters; a mandatory 180-degree flip is not claimed.

These are numerical and physical-consistency checks on synthetic parameters. No manufacturer data, held-out physical robot tests, 3D contact, collision sweep, vertical dynamics, tipping, tyre deformation, predictive shimmy or Gazebo run is claimed. M0 is retained as the ideal kinematic reference; drive calibration remains a separate next step.
