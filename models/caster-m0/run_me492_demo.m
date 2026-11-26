function manifest = run_me492_demo()
% Replays our own M0 Simulink model with shared SI JSON and command CSVs.
root = fileparts(mfilename('fullpath'));
p = jsondecode(fileread(fullfile(root,'parameters_si.json')));
assert(strcmp(p.units,'SI') && strcmp(p.parameter_status,'provisional_synthetic'));
assert(p.drive_radius_m>0 && p.caster_radius_m>0 && p.caster_trail_m>0 && p.drive_track_m>0);
for folder={'commands','results','plots','models'}
    if ~isfolder(fullfile(root,folder{1})), mkdir(fullfile(root,folder{1})); end
end
model = 'Me492CasterM0';
modelPath = fullfile(root,'models',[model '.slx']);
if bdIsLoaded(model)
    assert(strcmp(get_param(model,'Dirty'),'off'),'ME492:UnsavedModelEdits', ...
        'Save and close your manual model edits before replaying experiments.');
end
if ~isfile(modelPath), build_me492_model(); end
load_system(modelPath);
cleanup = onCleanup(@() close_unsaved_model(model));
mw = get_param(model,'ModelWorkspace');
fields = fieldnames(p);
for i=1:numel(fields), assignin(mw,fields{i},p.(fields{i})); end
assignin(mw,'caster_pivot_x_m',p.caster_pivot_m(1));
assignin(mw,'caster_pivot_y_m',p.caster_pivot_m(2));

% All angle values below are radians; degrees appear only in plots/teaching notes.
specs = {
    'straight_aligned',4,0.2,0,0;
    'straight_misaligned',4,0.2,0,pi/4;
    'reversal_perturbed',8,0.2,0,pi/180;
    'reversal_exact_aligned',8,0.2,0,0;
    'turn_in_place',4,0,0.5,0;
    'curved_turn',5,0.2,0.4,0
};
manifest = struct('schema_version',p.schema_version,'parameter_status',p.parameter_status, ...
    'model_level','M0_ideal_drive_with_passive_caster_kinematic_tracking', ...
    'physical_calibration',false,'caster_forces_feed_back_to_body',false, ...
    'H1_validated',false,'simulation_engine','Simulink', ...
    'matlab_version',version,'created_at_utc',char(datetime('now','TimeZone','UTC', ...
    'Format',"yyyy-MM-dd'T'HH:mm:ssXXX")), ...
    'input_interpolation','zero_order_hold','solver','ode4', ...
    'parameters',p,'runs',{{}});
allResults = cell(size(specs,1),1);
for i=1:size(specs,1)
    name = specs{i,1}; stop = specs{i,2};
    t = (0:p.command_sample_dt_s:stop)';
    v = specs{i,3}*ones(size(t)); omega = specs{i,4}*ones(size(t));
    if startsWith(name,'reversal'), v(t>=2)=-0.2; end
    commands = table(t,v,omega,'VariableNames', ...
        {'time_s','linear_velocity_mps','angular_velocity_radps'});
    commandFile = fullfile(root,'commands',[name '_commands.csv']);
    writetable(commands,commandFile);
    % Read the file back: these same bytes are the replay interface for Gazebo.
    commands = readtable(commandFile);
    assert(all(diff(commands.time_s)>0) && all(isfinite(table2array(commands)),'all'));
    assignin(mw,'command_array',table2array(commands));
    assignin(mw,'initial_x_m',0); assignin(mw,'initial_y_m',0);
    assignin(mw,'initial_yaw_rad',0); assignin(mw,'initial_caster_angle_rad',specs{i,5});
    set_param(model,'StopTime',num2str(stop));
    out = sim(model,'ReturnWorkspaceOutputs','on');
    q = values_at(out.get('me492_state'),commands.time_s);
    rates = values_at(out.get('me492_rates'),commands.time_s);
    theta = q(:,3); phi = q(:,4); psi = theta+phi;
    a = p.caster_pivot_m(1); b = p.caster_pivot_m(2);
    hingeX = q(:,1)+a*cos(theta)-b*sin(theta);
    hingeY = q(:,2)+a*sin(theta)+b*cos(theta);
    wheelX = hingeX-p.caster_trail_m*cos(psi);
    wheelY = hingeY-p.caster_trail_m*sin(psi);
    states = table(commands.time_s,commands.linear_velocity_mps,commands.angular_velocity_radps, ...
        q(:,1),q(:,2),theta,phi,psi,rates(:,1),rates(:,2),rates(:,3),rates(:,4), ...
        hingeX,hingeY,wheelX,wheelY,'VariableNames', ...
        {'time_s','linear_velocity_mps','angular_velocity_radps','x_m','y_m','yaw_rad', ...
        'caster_angle_body_rad','caster_angle_world_rad','caster_angle_body_rate_radps', ...
        'left_wheel_rate_radps','right_wheel_rate_radps','caster_wheel_rate_radps', ...
        'caster_hinge_x_m','caster_hinge_y_m','caster_wheel_center_x_m','caster_wheel_center_y_m'});
    assert(all(isfinite(table2array(states)),'all'),'ME492:NonfiniteOutput','Nonfinite output.');
    resultFile = fullfile(root,'results',[name '_states.csv']);
    writetable(states,resultFile); allResults{i}=states;
    scenario_plot(states,name,fullfile(root,'plots',[name '.png']));
    manifest.runs{end+1} = struct('scenario_id',name,'duration_s',stop, ...
        'initial_pose',struct('x_m',0,'y_m',0,'yaw_rad',0), ...
        'initial_caster_angle_body_rad',specs{i,5}, ...
        'commands_csv',['commands/' name '_commands.csv'], ...
        'states_csv',['results/' name '_states.csv'],'plot_png',['plots/' name '.png'], ...
        'row_count',height(states),'final_pose',[states.x_m(end),states.y_m(end),states.yaw_rad(end)], ...
        'final_caster_angle_body_rad',states.caster_angle_body_rad(end));
    fprintf('%s: %d rows, final=[%.6f %.6f %.6f], caster=%.6f rad\n', ...
        name,height(states),states.x_m(end),states.y_m(end),states.yaw_rad(end), ...
        states.caster_angle_body_rad(end));
end
summary_plot(allResults,fullfile(root,'plots','summary.png'));
% The saved model remains the default aligned case. Do not save replay mutations.
try
    print(['-s' model],'-dpng',fullfile(root,'plots','model_diagram.png'));
catch diagramError
    manifest.diagram_export_note = diagramError.message;
end
fid = fopen(fullfile(root,'run_manifest.json'),'w','n','UTF-8');
assert(fid>=0,'Cannot write run manifest.');
fwrite(fid,jsonencode(manifest,PrettyPrint=true),'char'); fclose(fid);
fprintf('Replay complete. Run verify_me492_model for numerical evidence.\n');
end

function values = values_at(series,t)
values = squeeze(series.Data);
if size(values,1)~=numel(series.Time), values=values.'; end
assert(size(values,1)==numel(series.Time),'ME492:UnexpectedLogShape','Unexpected log dimensions.');
values = interp1(series.Time,values,t,'linear');
end

function scenario_plot(s,name,file)
fig=figure('Visible','off','Color','w','Position',[100 100 1200 740]);
if isprop(fig,'Theme'), fig.Theme='light'; end
tiledlayout(fig,2,2,'TileSpacing','compact','Padding','compact');
nexttile; plot(s.x_m,s.y_m,'LineWidth',1.5); hold on;
plot(s.x_m(1),s.y_m(1),'go','MarkerFaceColor','g');
plot(s.x_m(end),s.y_m(end),'rx'); axis equal; grid on;
xlabel('world x (m)'); ylabel('world y (m)'); title('Ideal body path');
nexttile; plot(s.time_s,rad2deg(s.caster_angle_body_rad),'LineWidth',1.5); grid on;
xlabel('time (s)'); ylabel('caster angle relative to body (deg)'); title('Passive caster tracking');
nexttile; yyaxis left; stairs(s.time_s,s.linear_velocity_mps,'LineWidth',1.3); ylabel('v (m/s)');
yyaxis right; stairs(s.time_s,s.angular_velocity_radps,'LineWidth',1.3); ylabel('omega (rad/s)');
grid on; xlabel('time (s)'); title('Shared CSV commands: zero-order hold');
nexttile; plot(s.time_s,[s.left_wheel_rate_radps s.right_wheel_rate_radps s.caster_wheel_rate_radps], ...
    'LineWidth',1.2); grid on; xlabel('time (s)'); ylabel('wheel rate (rad/s)');
legend('left drive','right drive','caster rolling','Location','best'); title('Kinematic wheel rates');
sgtitle([strrep(name,'_',' ') ' | M0, synthetic parameters, no contact dynamics']);
exportgraphics(fig,file,'Resolution',150); close(fig);
end

function summary_plot(allResults,file)
fig=figure('Visible','off','Color','w','Position',[100 100 1300 820]);
if isprop(fig,'Theme'), fig.Theme='light'; end
tiledlayout(fig,2,2,'TileSpacing','compact','Padding','compact');
a=allResults{1}; m=allResults{2}; r=allResults{3}; e=allResults{4};
t=allResults{5}; c=allResults{6};
nexttile; plot(a.x_m,a.y_m,'LineWidth',1.5); hold on;
plot(m.x_m,m.y_m,'--','LineWidth',1.5); plot(c.x_m,c.y_m,'LineWidth',1.5); axis equal; grid on;
xlabel('world x (m)'); ylabel('world y (m)');
legend('aligned start','45 deg caster start: identical body path','curved turn','Location','best');
title('Caster tracking does not alter body motion in M0');
nexttile; plot(a.time_s,rad2deg(a.caster_angle_body_rad),'LineWidth',1.5); hold on;
plot(m.time_s,rad2deg(m.caster_angle_body_rad),'LineWidth',1.5); grid on;
xlabel('time (s)'); ylabel('body-relative caster angle (deg)');
legend('aligned','misaligned','Location','best'); title('Forward caster alignment');
nexttile; plot(r.time_s,rad2deg(r.caster_angle_body_rad),'LineWidth',1.5); hold on;
plot(e.time_s,rad2deg(e.caster_angle_body_rad),'--','LineWidth',1.5); xline(2,':','reverse starts');
grid on; xlabel('time (s)'); ylabel('body-relative caster angle (deg)');
legend('documented 1 deg initial perturbation','exact aligned ideal equilibrium','Location','best');
title('Reverse: initial state matters; no hidden angle jump');
nexttile; plot(t.time_s,rad2deg(t.caster_angle_body_rad),'LineWidth',1.5); hold on;
plot(t.time_s,rad2deg(t.caster_angle_world_rad),'LineWidth',1.5); grid on;
xlabel('time (s)'); ylabel('angle (deg)'); legend('relative to body','world heading','Location','best');
title('In-place turn: two different angle frames');
sgtitle('ME492 own Simulink model | M0 kinematics | SYNTHETIC, NOT A CALIBRATED DIGITAL TWIN');
exportgraphics(fig,file,'Resolution',150); close(fig);
end

function close_unsaved_model(model)
if bdIsLoaded(model), close_system(model,0); end
end
