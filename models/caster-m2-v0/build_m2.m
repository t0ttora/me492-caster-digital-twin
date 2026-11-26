function path = build_m2()
% Our own force-coupled Simulink model; no copied upstream SLX or decorative blocks.
root=fileparts(mfilename('fullpath')); [~,d,p]=m2_load_parameters(); addpath(root);
folder=fullfile(root,'models'); if ~isfolder(folder), mkdir(folder); end
model='Me492CasterM2'; path=fullfile(folder,[model '.slx']);
assert(~isfile(path),'ME492:PreserveM2','Rename your existing SLX before rebuilding.');
assert(~bdIsLoaded(model),'ME492:M2Loaded','Close the model before building.');
load_system('simulink'); new_system(model);
set_param(model,'SolverType','Fixed-step','Solver','ode4','FixedStep','solver_step_s', ...
    'StopTime','4','ReturnWorkspaceOutputs','on','SaveOutput','off','SignalLogging','off');
mw=get_param(model,'ModelWorkspace'); assignin(mw,'p_vector',p);
assignin(mw,'solver_step_s',d.solver_step_s); assignin(mw,'command_sample_dt_s',d.command_sample_dt_s);
assignin(mw,'caster_contact_enabled',1);
initial=zeros(13,1); initial(1:2)=d.body_com_m(1:2); assignin(mw,'initial_state',initial);
t=(0:d.command_sample_dt_s:4)'; torque=0.25*min(t/0.2,1); torque(t>=1)=0.06;
assignin(mw,'torque_array',[t torque torque]);
add_block('simulink/Sources/From Workspace',[model '/IndependentWheelTorques'], ...
    'VariableName','torque_array','SampleTime','command_sample_dt_s','Interpolate','off', ...
    'OutputAfterFinalValue','Holding final value','Position',[30 35 230 80]);
add_block('simulink/Sources/Constant',[model '/PhysicalParameters'], ...
    'Value','p_vector','Position',[30 315 230 355]);
add_block('simulink/Sources/Constant',[model '/CasterGroundContactEnabled'], ...
    'Value','caster_contact_enabled','Position',[30 390 230 430]);
add_block('simulink/Continuous/Integrator',[model '/MechanicalState'], ...
    'InitialCondition','initial_state','Position',[780 90 840 140]);
contact=[model '/WheelGroundForces']; dynamics=[model '/BodyCasterInertiaDynamics'];
add_block('simulink/User-Defined Functions/MATLAB Function',contact,'Position',[325 255 555 400]);
add_block('simulink/User-Defined Functions/MATLAB Function',dynamics,'Position',[580 30 735 190]);
chart=sfroot; cc=chart.find('-isa','Stateflow.EMChart','Path',contact);
cc.Script=sprintf('function [w,c] = contact(x,p,enabled)\n%%#codegen\n[w,c]=m2_contact(x,p,enabled);\nend\n');
dd=chart.find('-isa','Stateflow.EMChart','Path',dynamics);
dd.Script=sprintf('function [dx,o] = dynamics(x,tau,p,w)\n%%#codegen\n[dx,o]=m2_dynamics(x,tau(:),p,w);\nend\n');
set_param(contact,'BackgroundColor','yellow'); set_param(dynamics,'BackgroundColor','lightBlue');
record(model,'StateRecord','m2_states',[940 95 1100 135]);
record(model,'ForceRecord','m2_wrenches',[680 290 840 330]);
record(model,'ContactRecord','m2_contacts',[680 365 840 405]);
record(model,'EnergyPowerRecord','m2_energy',[940 195 1100 235]);
wire(model,'MechanicalState/1','WheelGroundForces/1');
wire(model,'PhysicalParameters/1','WheelGroundForces/2');
wire(model,'CasterGroundContactEnabled/1','WheelGroundForces/3');
wire(model,'MechanicalState/1','BodyCasterInertiaDynamics/1');
wire(model,'IndependentWheelTorques/1','BodyCasterInertiaDynamics/2');
wire(model,'PhysicalParameters/1','BodyCasterInertiaDynamics/3');
wire(model,'WheelGroundForces/1','BodyCasterInertiaDynamics/4');
wire(model,'BodyCasterInertiaDynamics/1','MechanicalState/1');
wire(model,'MechanicalState/1','StateRecord/1');
wire(model,'WheelGroundForces/1','ForceRecord/1');
wire(model,'WheelGroundForces/2','ContactRecord/1');
wire(model,'BodyCasterInertiaDynamics/2','EnergyPowerRecord/1');
note=Simulink.Annotation(model,sprintf(['M2: independent drive torques -> contact forces -> coupled acceleration -> state\n' ...
    '13 states: base COM pose, relative caster angle, 7 velocities, motor work and dissipation.\n' ...
    'Synthetic planar dynamics; gravity sets quasi-static loads. No vertical impact/tipping or robot validation.\n' ...
    'Double-click functions; readable equation sources: m2_contact.m, m2_dynamics.m, m2_mass_terms.m.']));
note.Position=[30 480]; save_system(model,path); fprintf('Own M2 model saved: %s\n',path);
end
function record(model,name,variable,position)
add_block('simulink/Sinks/To Workspace',[model '/' name],'VariableName',variable, ...
    'SaveFormat','Timeseries','Position',position);
end
function wire(model,source,destination)
add_line(model,source,destination,'autorouting','on');
end

