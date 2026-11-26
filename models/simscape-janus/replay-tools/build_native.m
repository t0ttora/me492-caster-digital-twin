function build_native(file,a,h,stop,logh)
if nargin<5,logh=h;end
root=fileparts(mfilename('fullpath'));source=fullfile(root,'..','evidence_export_20261116_013532','latest_alignment_replay','evidence_latest_alignment_replay.slx');
copyfile(source,file);load_system(file);[~,m]=fileparts(file);blocks=find_system(m,'LookUnderMasks','all','Type','Block');
sw=find_named(blocks,'J_SWIVEL');set_param(sw,'PositionTargetValue',num2str(a),'PositionTargetValueUnits','deg');
set_param(m,'SolverType','Fixed-step','Solver','ode4','FixedStep',num2str(h,16),'StopTime',num2str(stop,16),'SimscapeLogType','none');
% Explicit full-step retention including existing exports. Preserve physical dynamics.
sinks=find_system(m,'LookUnderMasks','all','BlockType','ToWorkspace');
for j=1:numel(sinks),set_param(sinks{j},'SampleTime',num2str(logh,16),'Decimation','1','MaxDataPoints','inf');end
for w={'L','R','C'}
 c=find_named(blocks,['CONTACT_' w{1}]);set_param(c,'SensePenetrationDepth','on');
 template=find_named(blocks,'Converter [N]');p=get_param(c,'Parent');
 cv=add_block(template,[p '/EXPORT_CONV_pen_' w{1}]);set_param(cv,'Unit','m');simscape.addConnection(c,'pen_out',cv,'input');add_export(cv,['pen_' w{1}],logh);
 switch w{1},case 'L',cn='Converter [N]2';case 'R',cn='Converter [N]1';otherwise,cn='Converter [N]';end
 add_export(find_named(blocks,cn),['normal_' w{1}],logh);
end
% Log actual signal feeding ideal axle torque inputs, not invented CSV constants.
add_export(find_named(blocks,'Constant'),'tau_left_Nm',logh);add_export(find_named(blocks,'Constant1'),'tau_right_Nm',logh);
% Existing reference frame is 75 mm below the axle midpoint. Add a massless
% rigid offset and independent World->midpoint pose sensing to the candidate.
rt=add_block(find_named(blocks,'RT_LIFT'),[m '/AXLE_MIDPOINT_OFFSET']);set_param(rt,'TranslationCartesianOffset','[0 0 0.075]');
sensor=add_block(find_named(blocks,'ROBOT_POSE_SENSOR'),[m '/AXLE_MIDPOINT_SENSOR']);
simscape.addConnection(find_named(blocks,'J_FREE'),'F',rt,'B');simscape.addConnection(rt,'F',sensor,'F');simscape.addConnection(find_named(blocks,'World Frame'),'W',sensor,'B');
ports={'x_out','y_out','seq_out'};labels={'axle_x_m','axle_y_m','axle_angles_deg'};units={'m','m','deg'};
for j=1:3
 cv=add_block(find_named(blocks,'CONV_ROBOT_X'),[m '/AXLE_CONV_' num2str(j)]);set_param(cv,'Unit',units{j});simscape.addConnection(sensor,ports{j},cv,'input');add_export(cv,labels{j},logh);
end
try,set_param(m,'SimMechanicsOpenEditorOnUpdate','off');catch,end
save_system(m,file,'SaveDirtyReferencedModels','off');
end
