function manifest = run_m2()
root=fileparts(mfilename('fullpath')); addpath(root); [g,d,p]=m2_load_parameters();
for f={'commands','results','plots','models'}, if ~isfolder(fullfile(root,f{1})), mkdir(fullfile(root,f{1})); end, end
model='Me492CasterM2'; modelPath=fullfile(root,'models',[model '.slx']);
if bdIsLoaded(model), assert(strcmp(get_param(model,'Dirty'),'off'),'Save manual model edits before replay.'); end
if ~isfile(modelPath), build_m2(); end
load_system(modelPath); cleanup=onCleanup(@() close_model(model)); mw=get_param(model,'ModelWorkspace');
manifest=struct('schema_version',d.schema_version,'parameter_status',d.parameter_status, ...
    'engine','Simulink','model','Me492CasterM2','matlab_version',version, ...
    'physical_robot_validation',false,'H1_validated_on_robot',false, ...
    'geometry',g,'dynamics',d,'normal_load_model',d.normal_load_model, ...
    'solver','ode4','command_interpolation','zero_order_hold', ...
    'created_at_utc',char(datetime('now','TimeZone','UTC','Format',"yyyy-MM-dd'T'HH:mm:ssXXX")), ...
    'runs',{{}});
% id, duration, initial phi, contact enable, step multiplier, initial condition kind
specs={
 'aligned_forward',4,0,1,1,'rest';
 'misaligned_forward',4,pi/4,1,1,'rest';
 'mirror_misaligned',4,-pi/4,1,1,'rest';
 'caster_contact_disabled',4,pi/4,0,1,'rest';
 'unequal_torque',4,0,1,1,'rest';
 'reversal_perturbed',7,pi/180,1,1,'rest';
 'zero_rest',2,pi/4,1,1,'rest';
 'passive_coast',5,pi/4,1,1,'coast';
 'conservative_free',3,0.6,0,1,'free';
 'misaligned_fine',4,pi/4,1,0.5,'rest';
 'misaligned_finer',4,pi/4,1,0.25,'rest';
 'reversal_no_swivel_resistance',7,pi/180,1,1,'rest'};
allResults=cell(size(specs,1),1);
for k=1:size(specs,1)
    name=specs{k,1}; stop=specs{k,2}; t=(0:d.command_sample_dt_s:stop)';
    common=0.25*min(t/0.2,1); common(t>=1)=0.06;
    tauL=common; tauR=common;
    if strcmp(name,'unequal_torque')
        tauL=0.15*min(t/0.2,1); tauR=0.25*min(t/0.2,1);
        tauL(t>=1.5)=0.02; tauR(t>=1.5)=0.06;
    elseif startsWith(name,'reversal_')
        tauL(t>=1.5)=-0.25; tauR=tauL;
    elseif any(strcmp(name,{'zero_rest','passive_coast','conservative_free'}))
        tauL(:)=0; tauR(:)=0;
    end
    commands=table(t,tauL,tauR,'VariableNames',{'time_s','left_drive_torque_Nm','right_drive_torque_Nm'});
    commandFile=fullfile(root,'commands',[name '_torque.csv']); writetable(commands,commandFile);
    commands=readtable(commandFile); x0=zeros(13,1); x0(1:2)=d.body_com_m(1:2); x0(4)=specs{k,3}; pk=p;
    if strcmp(specs{k,6},'coast')
        x0(5)=0.35; x0(7)=0.15; x0(8)=-0.1; x0(9:10)=0.35/g.drive_radius_m;
        [~,~,JC,~,~,~,~,eC,~]=m2_geometry(x0,pk); x0(11)=eC.'*(JC*x0(5:11))/g.caster_radius_m;
    elseif strcmp(specs{k,6},'free')
        x0(5:11)=[0.15;-0.05;0.3;-0.4;2;-1;3]; pk([17:28 31])=0;
    end
    if strcmp(name,'reversal_no_swivel_resistance'), pk([27 28 31])=0; end
    step=d.solver_step_s*specs{k,5};
    assignin(mw,'p_vector',pk); assignin(mw,'initial_state',x0); assignin(mw,'torque_array',table2array(commands));
    assignin(mw,'caster_contact_enabled',specs{k,4}); assignin(mw,'solver_step_s',step);
    assignin(mw,'command_sample_dt_s',d.command_sample_dt_s); set_param(model,'StopTime',num2str(stop));
    out=sim(model,'ReturnWorkspaceOutputs','on');
    q=at(out.get('m2_states'),t); w=at(out.get('m2_wrenches'),t);
    c=at(out.get('m2_contacts'),t); o=at(out.get('m2_energy'),t);
    s=make_table(t,commands,q,w,c,o,g,d); assert(all(isfinite(table2array(s)),'all'));
    resultFile=fullfile(root,'results',[name '_states.csv']); writetable(s,resultFile); allResults{k}=s;
    case_plot(s,name,fullfile(root,'plots',[name '.png']));
    manifest.runs{end+1}=struct('scenario_id',name,'duration_s',stop,'solver_step_s',step, ...
        'caster_tangential_contact_enabled',logical(specs{k,4}),'initial_state',x0.', ...
        'conservative_no_friction_override',strcmp(specs{k,6},'free'),'parameter_vector_used',pk.', ...
        'swivel_and_bore_resistance_disabled',strcmp(name,'reversal_no_swivel_resistance'), ...
        'commands_csv',['commands/' name '_torque.csv'],'states_csv',['results/' name '_states.csv'], ...
        'plot_png',['plots/' name '.png'],'row_count',height(s), ...
        'final_pose',[s.x_m(end) s.y_m(end) s.yaw_rad(end)], ...
        'final_caster_angle_body_rad',s.caster_angle_body_rad(end), ...
        'max_energy_balance_error_J',max(abs(s.energy_balance_error_J)));
    fprintf('%s h=%.7g: final pose=[%.6f %.6f %.6f], phi=%.6f, balance=%.3g J\n', ...
        name,step,s.x_m(end),s.y_m(end),s.yaw_rad(end),s.caster_angle_body_rad(end),max(abs(s.energy_balance_error_J)));
end
summary_plot(allResults,fullfile(root,'plots','summary.png'));
print(['-s' model],'-dpng','-r150',fullfile(root,'plots','model_diagram.png'));
write_json(fullfile(root,'run_manifest.json'),manifest);
fprintf('M2 Simulink replays complete. Physical consistency still requires verify_m2.\n');
end

function s=make_table(t,commands,q,w,c,o,g,d)
theta=q(:,3); phi=q(:,4); cosT=cos(theta); sinT=sin(theta);
baseX=q(:,1); baseY=q(:,2);
originX=baseX-cosT*d.body_com_m(1)+sinT*d.body_com_m(2);
originY=baseY-sinT*d.body_com_m(1)-cosT*d.body_com_m(2);
u=cosT.*q(:,5)+sinT.*q(:,6); v=-sinT.*q(:,5)+cosT.*q(:,6);
rx=g.caster_pivot_m(1)-d.body_com_m(1); ry=g.caster_pivot_m(2)-d.body_com_m(2);
psi=theta+phi; rHx=cosT*rx-sinT*ry; rHy=sinT*rx+cosT*ry;
casterX=baseX+rHx-g.caster_trail_m*cos(psi); casterY=baseY+rHy-g.caster_trail_m*sin(psi);
casterVx=q(:,5)-q(:,7).*rHy+g.caster_trail_m*(q(:,7)+q(:,8)).*sin(psi);
casterVy=q(:,6)+q(:,7).*rHx-g.caster_trail_m*(q(:,7)+q(:,8)).*cos(psi);
Px=d.body_lumped_mass_kg*q(:,5)+d.caster_lumped_mass_kg*casterVx;
Py=d.body_lumped_mass_kg*q(:,6)+d.caster_lumped_mass_kg*casterVy;
Hz=d.body_lumped_mass_kg*(baseX.*q(:,6)-baseY.*q(:,5))+d.body_yaw_inertia_kgm2*q(:,7) ...
    +d.caster_lumped_mass_kg*(casterX.*casterVy-casterY.*casterVx) ...
    +d.caster_swivel_inertia_com_kgm2*(q(:,7)+q(:,8));
balance=o(:,1)-o(1,1)+q(:,13)-q(:,12);
values=[t commands.left_drive_torque_Nm commands.right_drive_torque_Nm ...
    originX originY theta phi psi baseX baseY u v q(:,7:11) w(:,1:6) c(:,7:9) c(:,1:6) ...
    w(:,10:11) o(:,4) o(:,1:3) q(:,12:13) balance Px Py Hz];
names={'time_s','left_drive_torque_Nm','right_drive_torque_Nm','x_m','y_m','yaw_rad', ...
    'caster_angle_body_rad','caster_angle_world_rad','body_com_x_world_m','body_com_y_world_m', ...
    'body_long_velocity_mps','body_lat_velocity_mps','body_yaw_rate_radps', ...
    'caster_relative_rate_radps','left_wheel_rate_radps','right_wheel_rate_radps','caster_roll_rate_radps', ...
    'left_fx_world_N','left_fy_world_N','right_fx_world_N','right_fy_world_N','caster_fx_world_N','caster_fy_world_N', ...
    'normal_left_N','normal_right_N','normal_caster_N', ...
    'left_long_slip_mps','left_lat_slip_mps','right_long_slip_mps','right_lat_slip_mps', ...
    'caster_long_slip_mps','caster_lat_slip_mps','swivel_bearing_torque_Nm','bore_ground_torque_Nm', ...
    'caster_external_yaw_moment_Nm','kinetic_energy_J','motor_power_W','dissipation_power_W', ...
    'motor_work_J','dissipated_energy_J','energy_balance_error_J', ...
    'linear_momentum_x_kgmps','linear_momentum_y_kgmps','angular_momentum_z_kgm2ps'};
assert(size(values,2)==numel(names)); s=array2table(values,'VariableNames',names);
end
function v=at(series,t)
v=squeeze(series.Data); if size(v,1)~=numel(series.Time), v=v.'; end
assert(size(v,1)==numel(series.Time)); v=interp1(series.Time,v,t,'linear');
end
function fig=new_figure(position)
fig=figure('Visible','off','Color','w','Position',position); if isprop(fig,'Theme'), fig.Theme='light'; end
end
function case_plot(s,name,path)
fig=new_figure([100 100 1250 820]); tiledlayout(fig,2,2,'TileSpacing','compact');
nexttile; plot(s.x_m,s.y_m,'LineWidth',1.5); grid on; xlabel('axle origin world x (m)'); ylabel('world y (m)'); title('Torque-driven body trajectory');
nexttile; plot(s.time_s,rad2deg([s.yaw_rad s.caster_angle_body_rad]),'LineWidth',1.3); grid on;
xlabel('time (s)'); ylabel('angle (deg)'); legend('body yaw','relative caster angle','Location','best'); title('Force-coupled angles');
nexttile; yyaxis left; plot(s.time_s,[s.caster_fx_world_N s.caster_fy_world_N],'LineWidth',1.2); ylabel('caster force (N)');
yyaxis right; plot(s.time_s,s.caster_external_yaw_moment_Nm,'LineWidth',1.2); ylabel('caster external moment about base COM (Nm)');
xlabel('time (s)'); grid on; legend('world Fx','world Fy','external yaw moment','Location','best'); title('Actual contact feedback');
nexttile; plot(s.time_s,[s.kinetic_energy_J s.motor_work_J s.dissipated_energy_J],'LineWidth',1.2); grid on;
xlabel('time (s)'); ylabel('energy / work (J)'); legend('kinetic','motor work','dissipation','Location','best'); title('Energy accounting');
sgtitle([strrep(name,'_',' ') ' | M2 synthetic planar dynamics, no robot validation']);
exportgraphics(fig,path,'Resolution',150); close(fig);
end
function summary_plot(s,path)
fig=new_figure([100 100 1400 1150]); tiledlayout(fig,3,2,'TileSpacing','compact');
a=s{1}; m=s{2}; mir=s{3}; off=s{4}; rev=s{6}; coast=s{8}; fine=s{10}; finer=s{11}; noResistance=s{12};
nexttile; plot(a.x_m,a.y_m,m.x_m,m.y_m,off.x_m,off.y_m,'LineWidth',1.4); grid on;
xlabel('world x (m)'); ylabel('world y (m), expanded scale'); legend('aligned','45 deg caster','caster tangential effects off','Location','best'); title('Identical wheel torques, different body paths');
nexttile; plot(m.time_s,rad2deg([a.yaw_rad m.yaw_rad mir.yaw_rad off.yaw_rad]),'LineWidth',1.4); grid on;
xlabel('time (s)'); ylabel('body yaw (deg)'); legend('aligned','+45 deg','-45 deg mirror','tangential effects off','Location','best'); title('Caster influence and sign symmetry');
nexttile; yyaxis left; plot(m.time_s,[m.caster_fx_world_N m.caster_fy_world_N],'LineWidth',1.2); ylabel('world caster force (N)');
yyaxis right; plot(m.time_s,m.caster_external_yaw_moment_Nm,'LineWidth',1.2); ylabel('external yaw moment (Nm)'); grid on;
xlabel('time (s)'); legend('Fx','Fy','yaw moment','Location','best'); title('Force and moment fed back to dynamics');
nexttile; plot(m.time_s,rad2deg(m.caster_angle_body_rad),rev.time_s,rad2deg(rev.caster_angle_body_rad), ...
    noResistance.time_s,rad2deg(noResistance.caster_angle_body_rad),'LineWidth',1.4); grid on;
xlabel('time (s)'); ylabel('relative caster angle (deg)'); legend('forward 45 deg','reverse: nominal resistance', ...
    'reverse: no swivel/bore resistance','Location','best'); title('Reverse response depends on resistance');
nexttile; plot(coast.time_s,[coast.kinetic_energy_J coast.dissipated_energy_J coast.kinetic_energy_J+coast.dissipated_energy_J],'LineWidth',1.4); grid on;
xlabel('time (s)'); ylabel('energy (J)'); legend('kinetic','dissipated','sum (zero input)','Location','best'); title('Passive energy check');
nexttile; semilogy(m.time_s,max(abs(m.yaw_rad-fine.yaw_rad),1e-16), ...
    m.time_s,max(abs(fine.yaw_rad-finer.yaw_rad),1e-16),'LineWidth',1.4); grid on;
xlabel('time (s)'); ylabel('absolute yaw difference (rad)'); legend('h vs h/2','h/2 vs h/4','Location','best'); title('Time-step refinement');
sgtitle('ME492 M2 | torque-driven, force-coupled planar model | ALL PARAMETERS SYNTHETIC');
exportgraphics(fig,path,'Resolution',150); close(fig);
end
function write_json(path,value)
fid=fopen(path,'w','n','UTF-8'); assert(fid>=0); fwrite(fid,jsonencode(value,PrettyPrint=true),'char'); fclose(fid);
end
function close_model(model)
if bdIsLoaded(model), close_system(model,0); end
end
