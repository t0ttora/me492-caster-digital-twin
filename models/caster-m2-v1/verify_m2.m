function evidence = verify_m2(stage)
% Independent physical expectations, then checks on actual Simulink CSV runs.
if nargin==0, stage='all'; end
root=fileparts(mfilename('fullpath'));
assert(exist('m2_contact','file')==2,'ME492:M2MissingKernel', ...
    'Force-coupled M2 contact implementation has not been created.');
[g,d,p]=m2_load_parameters();
checks=struct('name',{},'passed',{},'value',{},'requirement',{});
x=zeros(13,1); x(1:2)=d.body_com_m(1:2);
[w,c]=m2_contact(x,p,1);
[dx,o]=m2_dynamics(x,[0;0],p,w);
check('rest_is_equilibrium',max(abs(dx))<1e-12,max(abs(dx)),'<1e-12');
check('rest_has_no_tangential_force',max(abs(w(1:6)))==0,max(abs(w(1:6))),'=0');
check('static_load_sums_to_weight',abs(sum(c(7:9))-(p(1)+p(2))*p(16))<1e-10, ...
    abs(sum(c(7:9))-(p(1)+p(2))*p(16)),'<1e-10 N');

% COM must remain strictly inside the instantaneous triangle for every swivel angle.
minLoad=Inf; minEig=Inf;
for phi=linspace(-pi,pi,361)
    x(4)=phi; [~,ci]=m2_contact(x,p,1);
    [M,~]=m2_mass_terms(x,p);
    minLoad=min(minLoad,min(ci(7:9))); minEig=min(minEig,min(eig(M)));
end
check('all_swivel_angles_have_positive_support_load',minLoad>5,minLoad,'>5 N');
check('mass_matrix_positive_definite',minEig>1e-5,minEig,'>1e-5 kg / kg m^2 (mixed coordinates)');

% A spinning drive wheel at stationary body must push forward and slow itself.
x(:)=0; x(1:2)=d.body_com_m(1:2); x(9)=100;
[w,c]=m2_contact(x,p,1); [dx,~]=m2_dynamics(x,[0;0],p,w);
check('drive_spin_produces_forward_ground_force',w(1)>0,w(1),'>0 N');
check('ground_reaction_slows_unpowered_spinning_wheel',dx(9)<0,dx(9),'<0 rad/s^2');
check('force_inside_coulomb_circle',norm(w(1:2))<=p(17)*c(7)+1e-10, ...
    norm(w(1:2))-p(17)*c(7),'<=0 N');

% Positive lateral slip is opposed by lateral force; force does negative slip work.
x(9)=0; x(5:6)=[0;0.2];
[w,c]=m2_contact(x,p,1);
check('lateral_friction_opposes_positive_slip',w(2)<0,w(2),'<0 N');
check('combined_contact_is_dissipative',w(1)*c(1)+w(2)*c(2)<=1e-12, ...
    w(1)*c(1)+w(2)*c(2),'<=0 W');

% Forward rolling with no slip should not produce traction in the regularized law.
x(:)=0; x(1:2)=d.body_com_m(1:2); x(5)=0.2;
x(9:10)=0.2/g.drive_radius_m; x(11)=0.2/g.caster_radius_m;
[w,~]=m2_contact(x,p,1);
check('zero_slip_has_zero_traction',max(abs(w(1:6)))<1e-10,max(abs(w(1:6))),'<1e-10 N');

% Arbitrary nonzero state: numerical derivative of an independently written energy.
x=[d.body_com_m(1);d.body_com_m(2);0.3;0.7;0.2;-0.1;0.4;-0.8;4;2;3;0;0];
[w,c]=m2_contact(x,p,1); tau=[0.12;-0.03];
[dx,o]=m2_dynamics(x,tau,p,w);
h=1e-6;
dEnergy=(independent_energy(x+h*dx,g,d)-independent_energy(x-h*dx,g,d))/(2*h);
powerError=abs(dEnergy-(tau.'*x(9:10)-o(3)));
check('instantaneous_energy_power_balance',powerError<1e-7,powerError,'<1e-7 W');
check('reported_energy_matches_independent_expression', ...
    abs(o(1)-independent_energy(x,g,d))<1e-12, ...
    abs(o(1)-independent_energy(x,g,d)),'<1e-12 J');
check('all_resistances_are_dissipative',o(3)>=-1e-12,o(3),'>=0 W');
patchBudget=norm([w(5:6);w(11)/p(31)]);
check('caster_traction_and_bore_share_friction_budget',patchBudget<=p(18)*c(9)+1e-10, ...
    patchBudget-p(18)*c(9),'<=0 N equivalent combined patch load');

% Caster disable means frictionless planar support, with mass/inertias retained.
[wOff,~]=m2_contact(x,p,0);
check('disabled_caster_has_no_ground_force_or_ground_resistance', ...
    max(abs(wOff([5 6 9 11])))==0,max(abs(wOff([5 6 9 11]))),'=0');
check('bearing_resistance_retained_in_ablation',wOff(10)==w(10),abs(wOff(10)-w(10)),'=0 Nm');

if ~strcmp(stage,'kernel')
    assert(isfile(fullfile(root,'models','Me492CasterM2V1.slx')),'ME492:M2MissingModel','Own M2 SLX absent.');
    manifest=jsondecode(fileread(fullfile(root,'run_manifest.json')));
    assert(numel(manifest.runs)==12,'ME492:IncompleteReplay','Run all 12 scenarios before full verification.');
    for k=1:numel(manifest.runs)
        rr=manifest.runs(k);
        assert(strcmp(m2_file_hash(fullfile(root,rr.states_csv)),rr.states_md5) ...
          && strcmp(m2_file_hash(fullfile(root,rr.commands_csv)),rr.commands_md5), ...
          'ME492:ReplayChanged','Replay CSV differs from the recorded generation.');
    end
    a=read_case('aligned_forward'); m=read_case('misaligned_forward');
    off=read_case('caster_contact_disabled'); mirror=read_case('mirror_misaligned');
    fine=read_case('misaligned_fine'); finer=read_case('misaligned_finer');
    ca=readtable(fullfile(root,'commands','aligned_forward_torque.csv'));
    cm=readtable(fullfile(root,'commands','misaligned_forward_torque.csv'));
    co=readtable(fullfile(root,'commands','caster_contact_disabled_torque.csv'));
    check('aligned_misaligned_identical_torque_inputs',isequal(ca,cm),double(~isequal(ca,cm)),'=0 differences');
    check('enabled_disabled_identical_torque_inputs',isequal(cm,co),double(~isequal(cm,co)),'=0 differences');
    check('symmetric_aligned_has_zero_yaw',max(abs(a.yaw_rad))<1e-10,max(abs(a.yaw_rad)),'<1e-10 rad');
    check('positive_equal_torque_moves_forward',a.x_m(end)>0.1,a.x_m(end),'>0.1 m');
    yawInfluence=max(abs(m.yaw_rad-a.yaw_rad));
    positionInfluence=max(hypot(m.x_m-a.x_m,m.y_m-a.y_m));
    check('caster_angle_changes_yaw_meaningfully',yawInfluence>1e-3,yawInfluence,'>0.001 rad');
    check('caster_angle_changes_trajectory_meaningfully',positionInfluence>1e-3,positionInfluence,'>0.001 m');
    ablationInfluence=max(abs(m.yaw_rad-off.yaw_rad));
    check('caster_ground_force_ablation_changes_yaw',ablationInfluence>1e-3,ablationInfluence,'>0.001 rad');
    check('caster_force_is_not_numeric_zero',max(hypot(m.caster_fx_world_N,m.caster_fy_world_N))>0.1, ...
        max(hypot(m.caster_fx_world_N,m.caster_fy_world_N)),'>0.1 N');
    check('mirror_yaw_sign_and_magnitude',max(abs(m.yaw_rad+mirror.yaw_rad))<1e-8, ...
        max(abs(m.yaw_rad+mirror.yaw_rad)),'<1e-8 rad');
    check('mirror_lateral_trajectory',max(abs(m.y_m+mirror.y_m))<1e-8, ...
        max(abs(m.y_m+mirror.y_m)),'<1e-8 m');
    turn=read_case('unequal_torque');
    check('greater_right_torque_turns_left',max(turn.yaw_rad)>0.05,max(turn.yaw_rad),'>0.05 rad');
    reverse=read_case('reversal_perturbed');
    check('reverse_torque_eventually_moves_backward',reverse.body_long_velocity_mps(end)<-0.05, ...
        reverse.body_long_velocity_mps(end),'<-0.05 m/s');
    % A flip is not a universal invariant of force-coupled, resistive, torque-driven motion.
    check('reverse_caster_can_roll_backward',reverse.caster_roll_rate_radps(end)<-1, ...
        reverse.caster_roll_rate_radps(end),'<-1 rad/s');
    noResistance=read_case('reversal_no_swivel_resistance');
    reverseCommands=readtable(fullfile(root,'commands','reversal_perturbed_torque.csv'));
    freeSwivelCommands=readtable(fullfile(root,'commands','reversal_no_swivel_resistance_torque.csv'));
    check('reverse_sensitivity_uses_identical_torque_input',isequal(reverseCommands,freeSwivelCommands), ...
        double(~isequal(reverseCommands,freeSwivelCommands)),'=0 differences');
    reverseAngleSensitivity=max(abs(reverse.caster_angle_body_rad-noResistance.caster_angle_body_rad));
    check('reverse_swivel_resistance_changes_response',reverseAngleSensitivity>0.01,reverseAngleSensitivity,'>0.01 rad');
    rest=read_case('zero_rest');
    check('zero_torque_rest_simulation_stays_at_rest',max(hypot(rest.x_m,rest.y_m))<1e-12, ...
        max(hypot(rest.x_m,rest.y_m)),'<1e-12 m');
    coast=read_case('passive_coast');
    check('passive_energy_never_increases',max(diff(coast.kinetic_energy_J))<1e-9, ...
        max(diff(coast.kinetic_energy_J)),'<1e-9 J per logged sample');
    check('passive_energy_decays',coast.kinetic_energy_J(end)<0.1*coast.kinetic_energy_J(1), ...
        coast.kinetic_energy_J(end)/coast.kinetic_energy_J(1),'<0.1 initial energy');
    free=read_case('conservative_free');
    check('force_free_energy_conserved',max(abs(free.kinetic_energy_J-free.kinetic_energy_J(1)))<1e-8, ...
        max(abs(free.kinetic_energy_J-free.kinetic_energy_J(1))),'<1e-8 J');
    momentumError=max(abs(free{:,{'linear_momentum_x_kgmps','linear_momentum_y_kgmps','angular_momentum_z_kgm2ps'}}- ...
        free{1,{'linear_momentum_x_kgmps','linear_momentum_y_kgmps','angular_momentum_z_kgm2ps'}}),[],'all');
    check('force_free_linear_and_angular_momentum_conserved',momentumError<1e-8,momentumError,'<1e-8 SI');
    coarseFine=max(abs(m.yaw_rad-fine.yaw_rad)); fineFiner=max(abs(fine.yaw_rad-finer.yaw_rad));
    check('step_refinement_small_yaw_error',coarseFine<1e-4,coarseFine,'<0.0001 rad');
    check('step_refinement_converges',fineFiner<=max(coarseFine*0.3,1e-10),fineFiner,'<=0.3 coarse/fine error or 1e-10 rad');
    trajectoryError=max(hypot(m.x_m-fine.x_m,m.y_m-fine.y_m));
    casterAngleError=max(abs(m.caster_angle_body_rad-fine.caster_angle_body_rad));
    rateNames={'body_yaw_rate_radps','caster_relative_rate_radps','left_wheel_rate_radps','right_wheel_rate_radps','caster_roll_rate_radps'};
    rateError=max(abs(m{:,rateNames}-fine{:,rateNames}),[],'all');
    check('step_refinement_trajectory',trajectoryError<1e-4,trajectoryError,'<1e-4 m');
    check('step_refinement_relative_caster_angle',casterAngleError<1e-4,casterAngleError,'<1e-4 rad');
    check('step_refinement_all_rotational_rates',rateError<1e-4,rateError,'<1e-4 rad/s');
    check('physical_influence_exceeds_step_error',yawInfluence>100*max(coarseFine,1e-10), ...
        yawInfluence/max(coarseFine,1e-10),'>100 times numerical step difference');
    names={'aligned_forward','misaligned_forward','mirror_misaligned','caster_contact_disabled', ...
        'unequal_torque','reversal_perturbed','zero_rest','passive_coast','conservative_free', ...
        'misaligned_fine','misaligned_finer','reversal_no_swivel_resistance'};
    maxBalance=0; minDiss=Inf; minNormal=Inf; maxForceExcess=-Inf; maxPatchExcess=-Inf;
    for k=1:numel(names)
        s=read_case(names{k});
        check([names{k} '_finite'],all(isfinite(table2array(s)),'all'),double(any(~isfinite(table2array(s)),'all')),'=0 nonfinite');
        maxBalance=max(maxBalance,max(abs(s.energy_balance_error_J)));
        minDiss=min(minDiss,min(s.dissipation_power_W));
        minNormal=min(minNormal,min(s{:,{'normal_left_N','normal_right_N','normal_caster_N'}},[],'all'));
        if ~strcmp(names{k},'conservative_free')
            maxForceExcess=max(maxForceExcess,max([hypot(s.left_fx_world_N,s.left_fy_world_N)-p(17)*s.normal_left_N; ...
                hypot(s.right_fx_world_N,s.right_fy_world_N)-p(17)*s.normal_right_N; ...
                hypot(s.caster_fx_world_N,s.caster_fy_world_N)-p(18)*s.normal_caster_N]));
            if ~strcmp(names{k},'reversal_no_swivel_resistance')
                patchEquivalent=sqrt(s.caster_fx_world_N.^2+s.caster_fy_world_N.^2+(s.bore_ground_torque_Nm/p(31)).^2);
                maxPatchExcess=max(maxPatchExcess,max(patchEquivalent-p(18)*s.normal_caster_N));
            end
        end
    end
    check('all_run_integrated_energy_balance',maxBalance<1e-5,maxBalance,'<1e-5 J');
    check('all_run_dissipation_nonnegative',minDiss>=-1e-10,minDiss,'>=-1e-10 W');
    check('all_run_positive_support_loads',minNormal>5,minNormal,'>5 N');
    check('all_run_friction_circle_respected',maxForceExcess<1e-8,maxForceExcess,'<1e-8 N above limit');
    check('all_run_shared_caster_patch_budget',maxPatchExcess<1e-8,maxPatchExcess,'<1e-8 N equivalent above limit');
end
evidence=struct('stage',stage,'all_passed',all([checks.passed]),'checks',checks, ...
    'scope','Numerical/physical consistency of synthetic planar M2, not robot validation.', ...
    'checked_at_utc',char(datetime('now','TimeZone','UTC','Format',"yyyy-MM-dd'T'HH:mm:ssXXX")));
if strcmp(stage,'kernel'), filename='kernel_verification_evidence.json'; else, filename='verification_evidence.json'; end
fid=fopen(fullfile(root,filename),'w','n','UTF-8'); assert(fid>=0);
fwrite(fid,jsonencode(evidence,PrettyPrint=true),'char'); fclose(fid);
fprintf('M2 verification: %d/%d checks passed.\n',sum([checks.passed]),numel(checks));
assert(evidence.all_passed,'ME492:M2VerificationFailed','One or more M2 checks failed.');
    function check(name,passed,value,requirement)
        checks(end+1)=struct('name',name,'passed',logical(passed),'value',value,'requirement',requirement);
        fprintf('%s: passed=%d value=%.9g %s\n',name,passed,value,requirement);
    end
    function s=read_case(name)
        s=readtable(fullfile(root,'results',[name '_states.csv']));
    end
end

function T=independent_energy(x,g,d)
% Written directly from point velocities, not the implementation mass matrix.
theta=x(3); psi=x(3)+x(4); omega=x(7); relative=x(8);
rx=g.caster_pivot_m(1)-d.body_com_m(1); ry=g.caster_pivot_m(2)-d.body_com_m(2);
hingeVelocity=x(5:6)+omega*[-sin(theta)*rx-cos(theta)*ry;cos(theta)*rx-sin(theta)*ry];
casterVelocity=hingeVelocity+g.caster_trail_m*(omega+relative)*[sin(psi);-cos(psi)];
T=0.5*d.body_lumped_mass_kg*sum(x(5:6).^2)+0.5*d.caster_lumped_mass_kg*sum(casterVelocity.^2) ...
  +0.5*d.body_yaw_inertia_kgm2*omega^2+0.5*d.caster_swivel_inertia_com_kgm2*(omega+relative)^2 ...
  +0.5*d.drive_roll_inertia_kgm2*sum(x(9:10).^2)+0.5*d.caster_roll_inertia_kgm2*x(11)^2;
end
