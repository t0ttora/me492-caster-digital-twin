function run_mac_replay
root=fileparts(mfilename('fullpath'));cd(root);diary('mac_replay_diary.log');
Simulink.fileGenControl('set','CacheFolder',fullfile(root,'cache'),'CodeGenFolder',fullfile(root,'codegen'),'createDir',true);
ids={'M04-python-nominal','M05-python-cw','Q02-python-left','Q06-python-nominal','M01-python-nominal','Q01-python-nominal','M08-python-caster-plus30','M08-python-caster-minus30'};
for j=1:numel(ids)
 id=ids{j};srcdir=fullfile(root,'mac-inputs','J01',id);folder=fullfile(root,'mac_replays',id);if ~exist(folder,'dir'),mkdir(folder);end
 if exist(fullfile(folder,'report.json'),'file'),r=jsondecode(fileread(fullfile(folder,'report.json')));if r.completed,status_event(['Skip completed ' id]);continue;end;end
 m=['native_mac_' strrep(id,'-','_')];report=struct('completed',false,'kind','APPLIED_TORQUE_REPLAY_NATIVE_OPEN_LOOP','source_run_id',id,'solver','ode4','step_s',25e-6,'sensor_log_s',.001,'zero_order_hold',true,'error','');timer=tic;
 try
  cmdfile=fullfile(srcdir,'applied_torque.csv');cmd=readtable(cmdfile);assert(isequal(cmd.Properties.VariableNames,{'time_s','tau_left_Nm','tau_right_Nm'}));assert(cmd.time_s(1)==0&&all(diff(cmd.time_s)>0)&&all(isfinite(table2array(cmd)),'all'));
  sm=jsondecode(fileread(fullfile(srcdir,'manifest.json')));sc=jsondecode(fileread(fullfile(srcdir,'scenario.json')));report.command_sha256=file_sha256(cmdfile);assert(strcmp(report.command_sha256,sm.files.applied_torque_csv),'Source command hash mismatch');
  report.scenario_sha256=file_sha256(fullfile(srcdir,'scenario.json'));report.source_manifest_sha256=file_sha256(fullfile(srcdir,'manifest.json'));report.code_sha256=file_sha256([mfilename('fullpath') '.m']);
  report.source_initial_state=sm.initial_solver_state;report.requested_initial_axle_pose=sm.initial_axle_pose;report.initial_caster_deg=sc.caster_deg;report.seed=sc.seed;report.duration_s=cmd.time_s(end);
  report.native_difference_note='Native 3D saved contact model retained: 5 mm initial ground clearance, gravity settling, SmoothStickSlip mu_static .5/mu_dynamic .3, critical velocity .01 m/s; Python uses planar regularized slip/support model. Same applied effort is not physical-model equivalence. Initial horizontal pose/yaw/caster and zero rates matched. No source dynamics retuned.';
  file=fullfile(folder,[m '.slx']);build_native(file,sc.caster_deg,25e-6,cmd.time_s(end),.001);bs=find_system(m,'LookUnderMasks','all','Type','Block');jf=find_named(bs,'J_FREE');
  set_param(jf,'PxPositionTargetValue',num2str(sm.initial_axle_pose.x_m,17),'PyPositionTargetValue',num2str(sm.initial_axle_pose.y_m,17),'SphPositionTargetRotationMethod','StandardAxis','SphPositionTargetRotationStandardAxis','+Z','SphPositionTargetRotationAngle',num2str(sm.initial_axle_pose.yaw_rad*180/pi,17),'SphPositionTargetRotationAngleUnits','deg');
  for w={'L','R'},b=find_named(bs,[w{1} '_JOINT']);set_param(b,'VelocityTargetSpecify','on','VelocityTargetValue','0','VelocityTargetValueUnits','rad/s');end
  mw=get_param(m,'ModelWorkspace');assignin(mw,'axle_command_matrix',table2array(cmd));
  add_block('simulink/Sources/From Workspace',[m '/AXLE_ZOH_TRACE'],'VariableName','axle_command_matrix','Interpolate','off','OutputAfterFinalValue','Holding final value','SampleTime','0');
  dm=add_block('simulink/Signal Routing/Demux',[m '/AXLE_COMMAND_SPLIT'],'Outputs','2');add_line(m,'AXLE_ZOH_TRACE/1','AXLE_COMMAND_SPLIT/1');wire_axle_inputs(m,dm,[1 2]);save_system(m,file);report.model_sha256=file_sha256(file);
  copyfile(cmdfile,fullfile(folder,'commands.csv'));write_json(fullfile(folder,'manifest_initial.json'),report);status_event(['Running same-effort native replay ' id]);
  in=Simulink.SimulationInput(m);in=in.setModelParameter('ReturnWorkspaceOutputs','on','SaveTime','off','Timeout',1800);out=sim(in);report.error=out.ErrorMessage;
  save(fullfile(folder,'result.mat'),'out','report','-v7.3');T=export_native(out,folder,.001);report.end_s=T.time_s(end);report.completed=abs(report.end_s-cmd.time_s(end))<1e-8&&isempty(report.error);
  tq=min(cmd.time_s(end),T.time_s+1e-10);expected=interp1(cmd.time_s,[cmd.tau_left_Nm cmd.tau_right_Nm],tq,'previous');actual=[T.tau_left_Nm_1 T.tau_right_Nm_1];delta=actual-expected;
  report.applied_command_max_abs_error_Nm=max(abs(delta),[],'all');report.applied_command_equivalence_pass=report.applied_command_max_abs_error_Nm<=1e-10;
  writetable(table(T.time_s,expected(:,1),actual(:,1),delta(:,1),expected(:,2),actual(:,2),delta(:,2),'VariableNames',{'time_s','expected_left_Nm','actual_left_Nm','delta_left_Nm','expected_right_Nm','actual_right_Nm','delta_right_Nm'}),fullfile(folder,'applied_command_equivalence.csv'));
  report.measured_initial=struct('x_m',T.axle_x_m_1(1),'y_m',T.axle_y_m_1(1),'yaw_rad',T.axle_angles_deg_1(1)*pi/180,'caster_rad',T.caster_angle_1(1)*pi/180,'left_wheel_radps',T.left_speed_1(1),'right_wheel_radps',T.right_speed_1(1),'swivel_radps',T.swivel_speed_1(1));
  report.final=struct('x_m',T.axle_x_m_1(end),'y_m',T.axle_y_m_1(end),'yaw_rad',T.axle_angles_deg_1(end)*pi/180,'caster_rad',T.caster_angle_1(end)*pi/180);
 catch err,report.error=getReport(err,'extended','hyperlinks','off');disp(report.error);end
 report.wall_s=toc(timer);write_json(fullfile(folder,'report.json'),report);status_event(sprintf('MAC %s completed=%d',id,report.completed));if bdIsLoaded(m),close_system(m,0);end;clear out T
end
diary off
end
