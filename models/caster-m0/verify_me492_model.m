function evidence = verify_me492_model()
% Independent checks: catches missing motion, wrong signs/frame and non-replayable files.
root = fileparts(mfilename('fullpath'));
assert(isfile(fullfile(root,'models','Me492CasterM0.slx')), ...
    'ME492:MissingModel','Own Me492CasterM0 Simulink model has not been created.');
assert(isfile(fullfile(root,'results','straight_aligned_states.csv')), ...
    'ME492:MissingReplay','Scenario outputs have not been created.');
checks = struct('name',{},'passed',{},'error',{},'tolerance',{});

% v=0.2 m/s for 4 s: x=0.8 m. No yaw or caster change in exact aligned motion.
s = readtable(fullfile(root,'results','straight_aligned_states.csv'));
record('straight_final_x',abs(s.x_m(end)-0.8),1e-7);
record('straight_zero_yaw',max(abs(s.yaw_rad)),1e-9);
record('straight_aligned_caster',max(abs(s.caster_angle_body_rad)),1e-9);
record('drive_wheel_rate',max(abs(s.left_wheel_rate_radps-8/3)),1e-9);

% Pure translation admits an exact caster solution, independent of Simulink blocks.
s = readtable(fullfile(root,'results','straight_misaligned_states.csv'));
exactPhi = 2*atan(tan(pi/8)*exp(-4*s.time_s));
record('misaligned_exact_caster_solution',max(abs(s.caster_angle_body_rad-exactPhi)),2e-7);
record('no_caster_feedback_to_pose_M0',abs(s.x_m(end)-0.8),1e-7);

% v=0.2, omega=0.4 for 5 s: circle radius=0.5 m, yaw=2 rad.
s = readtable(fullfile(root,'results','curved_turn_states.csv'));
record('circle_x',abs(s.x_m(end)-0.5*sin(2)),1e-7);
record('circle_y',abs(s.y_m(end)-0.5*(1-cos(2))),1e-7);
record('circle_yaw',abs(s.yaw_rad(end)-2),1e-8);

% At phi=0, rear hinge a=-0.18, trail=0.05, omega=0.5:
% absolute caster rate=-1.8; body-relative rate=-2.3 rad/s. Missing -omega fails.
s = readtable(fullfile(root,'results','turn_in_place_states.csv'));
record('body_relative_caster_rate',abs(s.caster_angle_body_rate_radps(1)+2.3),1e-9);
record('turn_in_place_zero_translation',max(hypot(s.x_m,s.y_m)),1e-9);
record('turn_in_place_yaw',abs(s.yaw_rad(end)-2),1e-8);

% Two seconds forward then six seconds reverse: signed travel=-0.8 m.
s = readtable(fullfile(root,'results','reversal_perturbed_states.csv'));
record('reversal_signed_distance',abs(s.x_m(end)+0.8),1e-4);
exactFinal = 2*atan(tan(pi/360)*exp(16));
record('reversal_caster_solution',abs(s.caster_angle_body_rad(end)-exactFinal),2e-6);
record('reversal_documented_initial_perturbation',abs(s.caster_angle_body_rad(1)-pi/180),1e-12);
s = readtable(fullfile(root,'results','reversal_exact_aligned_states.csv'));
record('reverse_unstable_equilibrium_is_not_hidden',max(abs(s.caster_angle_body_rad)),1e-9);

% CSV replay contract: aligned and misaligned starts share exactly the same commands.
c0 = readtable(fullfile(root,'commands','straight_aligned_commands.csv'));
c1 = readtable(fullfile(root,'commands','straight_misaligned_commands.csv'));
record('same_commands_different_initial_states',max(abs(table2array(c0)-table2array(c1)),[],'all'),0);
record('strictly_increasing_command_clock',double(any(diff(c0.time_s)<=0)),0);
aligned = readtable(fullfile(root,'results','straight_aligned_states.csv'));
misaligned = readtable(fullfile(root,'results','straight_misaligned_states.csv'));
record('same_result_clock_for_pose_comparison',max(abs(aligned.time_s-misaligned.time_s)),0);
poseColumns = {'x_m','y_m','yaw_rad'};
record('entire_body_trajectory_independent_of_caster_M0', ...
    max(abs(aligned{:,poseColumns}-misaligned{:,poseColumns}),[],'all'),1e-12);

names = {'straight_aligned','straight_misaligned','reversal_perturbed', ...
    'reversal_exact_aligned','turn_in_place','curved_turn'};
for i=1:numel(names)
    s = readtable(fullfile(root,'results',[names{i} '_states.csv']));
    record([names{i} '_finite_states'],double(any(~isfinite(table2array(s)),'all')),0);
    record([names{i} '_increasing_result_clock'],double(any(diff(s.time_s)<=0)),0);
end

evidence = struct('all_passed',all([checks.passed]),'checks',checks, ...
    'scope','Numerical verification of M0 kinematics only. No physical calibration or H1 validation.', ...
    'checked_at_utc',char(datetime('now','TimeZone','UTC','Format',"yyyy-MM-dd'T'HH:mm:ssXXX")));
fid = fopen(fullfile(root,'verification_evidence.json'),'w','n','UTF-8');
assert(fid>=0,'Cannot write verification evidence.');
fwrite(fid,jsonencode(evidence,PrettyPrint=true),'char'); fclose(fid);
fprintf('ME492 verification: %d/%d checks passed.\n',sum([checks.passed]),numel(checks));
assert(evidence.all_passed,'ME492:VerificationFailed','One or more numerical checks failed.');

    function record(name,errorValue,tolerance)
        passed = isfinite(errorValue) && errorValue<=tolerance;
        checks(end+1) = struct('name',name,'passed',passed,'error',errorValue,'tolerance',tolerance);
        fprintf('%s: passed=%d error=%.9g tolerance=%.9g\n',name,passed,errorValue,tolerance);
    end
end
