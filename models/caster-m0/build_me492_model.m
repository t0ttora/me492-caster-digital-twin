function modelPath = build_me492_model()
% Create our own elementary-block M0 model. No upstream SLX is copied.
% Refuses to overwrite a model: preserve any later manual teaching edits.
root = fileparts(mfilename('fullpath'));
p = jsondecode(fileread(fullfile(root,'parameters_si.json')));
assert(p.drive_radius_m>0 && p.caster_radius_m>0 && p.caster_trail_m>0, ...
    'ME492:InvalidGeometry','Wheel radii and caster trail must be positive.');
assert(p.drive_track_m>0 && p.solver_step_s>0 && p.command_sample_dt_s>0);
assert(abs(p.command_sample_dt_s/p.solver_step_s-round(p.command_sample_dt_s/p.solver_step_s))<1e-9);
modelDir = fullfile(root,'models');
if ~isfolder(modelDir), mkdir(modelDir); end
model = 'Me492CasterM0';
modelPath = fullfile(modelDir,[model '.slx']);
assert(~isfile(modelPath),'ME492:PreserveExistingModel', ...
    'Model already exists. Rename it explicitly before rebuilding; manual edits are preserved.');
assert(~bdIsLoaded(model),'ME492:ModelAlreadyLoaded','Close existing model before building.');
load_system('simulink'); new_system(model);
set_param(model,'SolverType','Fixed-step','Solver','ode4', ...
    'FixedStep','solver_step_s','StopTime','4','ReturnWorkspaceOutputs','on', ...
    'SaveTime','on','TimeSaveName','tout','SaveOutput','off', ...
    'SignalLogging','off');
mw = get_param(model,'ModelWorkspace');
fields = fieldnames(p);
for i=1:numel(fields), assignin(mw,fields{i},p.(fields{i})); end
assignin(mw,'caster_pivot_x_m',p.caster_pivot_m(1));
assignin(mw,'caster_pivot_y_m',p.caster_pivot_m(2));
assignin(mw,'initial_x_m',0); assignin(mw,'initial_y_m',0);
assignin(mw,'initial_yaw_rad',0); assignin(mw,'initial_caster_angle_rad',0);
t = (0:p.command_sample_dt_s:4)';
assignin(mw,'command_array',[t 0.2*ones(size(t)) zeros(size(t))]);

add_block('simulink/Sources/From Workspace',[model '/MotionCommands'], ...
    'VariableName','command_array','SampleTime','command_sample_dt_s', ...
    'Interpolate','off','OutputAfterFinalValue','Holding final value', ...
    'Position',[40 140 195 175]);
add_block('simulink/Signal Routing/Demux',[model '/SplitCommands'], ...
    'Outputs','2','Position',[240 128 245 197]);
pose = [model '/IdealBodyPose'];
caster = [model '/PassiveCasterTracking'];
wheels = [model '/DriveWheelRates'];
blank(pose,[330 35 510 115]); build_pose(pose);
blank(caster,[330 155 510 245]); build_caster(caster);
blank(wheels,[330 290 510 360]); build_wheels(wheels);
set_param(pose,'BackgroundColor','lightBlue');
set_param(caster,'BackgroundColor','yellow');
add_block('simulink/Signal Routing/Mux',[model '/StateVector'], ...
    'Inputs','2','Position',[590 70 595 195]);
add_block('simulink/Signal Routing/Mux',[model '/RateVector'], ...
    'Inputs','3','Position',[590 225 595 345]);
add_block('simulink/Sinks/To Workspace',[model '/StateRecord'], ...
    'VariableName','me492_state','SaveFormat','Timeseries','Position',[675 65 805 100]);
add_block('simulink/Sinks/To Workspace',[model '/RateRecord'], ...
    'VariableName','me492_rates','SaveFormat','Timeseries','Position',[675 255 805 290]);
add_block('simulink/Sinks/Scope',[model '/StateScope'],'Position',[685 135 755 175]);
wire(model,'MotionCommands/1','SplitCommands/1');
for sub={'IdealBodyPose','PassiveCasterTracking','DriveWheelRates'}
    wire(model,'SplitCommands/1',[sub{1} '/1']);
    wire(model,'SplitCommands/2',[sub{1} '/2']);
end
wire(model,'IdealBodyPose/1','StateVector/1');
wire(model,'PassiveCasterTracking/1','StateVector/2');
wire(model,'PassiveCasterTracking/2','RateVector/1');
wire(model,'DriveWheelRates/1','RateVector/2');
wire(model,'PassiveCasterTracking/3','RateVector/3');
wire(model,'StateVector/1','StateRecord/1');
wire(model,'StateVector/1','StateScope/1');
wire(model,'RateVector/1','RateRecord/1');
a = Simulink.Annotation(model,sprintf(['M0: prescribed ideal body motion + passive caster kinematic tracking\n' ...
    'Synthetic parameters. No contact forces, motor dynamics, yaw disturbance or H1 validation.\n' ...
    'States: x, y, yaw, caster angle relative to body. Units: m, rad, s.']));
a.Position = [35 400];
save_system(model,modelPath);
fprintf('Own model saved: %s\n',modelPath);
end

function build_pose(s)
inport(s,'v',1,[25 45 55 65]); inport(s,'omega',2,[25 160 55 180]);
block(s,'Yaw','simulink/Continuous/Integrator',[120 155 160 185], ...
    'InitialCondition','initial_yaw_rad');
block(s,'CosYaw','simulink/Math Operations/Trigonometric Function',[210 110 250 140], ...
    'Operator','cos');
block(s,'SinYaw','simulink/Math Operations/Trigonometric Function',[210 205 250 235], ...
    'Operator','sin');
block(s,'XRate','simulink/Math Operations/Product',[310 40 345 75]);
block(s,'YRate','simulink/Math Operations/Product',[310 210 345 245]);
block(s,'X','simulink/Continuous/Integrator',[390 40 430 75],'InitialCondition','initial_x_m');
block(s,'Y','simulink/Continuous/Integrator',[390 210 430 245],'InitialCondition','initial_y_m');
block(s,'Pose','simulink/Signal Routing/Mux',[485 80 490 210],'Inputs','3');
outport(s,'pose',1,[550 130 580 150]);
wire(s,'omega/1','Yaw/1'); wire(s,'Yaw/1','CosYaw/1'); wire(s,'Yaw/1','SinYaw/1');
wire(s,'v/1','XRate/1'); wire(s,'CosYaw/1','XRate/2'); wire(s,'XRate/1','X/1');
wire(s,'v/1','YRate/1'); wire(s,'SinYaw/1','YRate/2'); wire(s,'YRate/1','Y/1');
wire(s,'X/1','Pose/1'); wire(s,'Y/1','Pose/2'); wire(s,'Yaw/1','Pose/3');
wire(s,'Pose/1','pose/1');
end

function build_caster(s)
inport(s,'v',1,[20 35 50 55]); inport(s,'omega',2,[20 170 50 190]);
block(s,'OmegaTimesY','simulink/Math Operations/Gain',[95 100 165 130], ...
    'Gain','caster_pivot_y_m');
block(s,'HingeVx','simulink/Math Operations/Sum',[210 35 240 75],'Inputs','+-');
block(s,'HingeVy','simulink/Math Operations/Gain',[100 170 170 200], ...
    'Gain','caster_pivot_x_m');
block(s,'Phi','simulink/Continuous/Integrator',[580 150 620 185], ...
    'InitialCondition','initial_caster_angle_rad');
block(s,'SinPhi','simulink/Math Operations/Trigonometric Function',[290 95 330 125], ...
    'Operator','sin');
block(s,'CosPhi','simulink/Math Operations/Trigonometric Function',[290 225 330 255], ...
    'Operator','cos');
block(s,'VxSinPhi','simulink/Math Operations/Product',[375 45 410 80]);
block(s,'VyCosPhi','simulink/Math Operations/Product',[375 140 410 175]);
block(s,'LateralHingeVelocity','simulink/Math Operations/Sum',[455 75 485 120], ...
    'Inputs','-+');
block(s,'AbsoluteCasterRate','simulink/Math Operations/Gain',[510 70 575 100], ...
    'Gain','1/caster_trail_m');
block(s,'BodyRelativeRate','simulink/Math Operations/Sum',[660 65 690 105], ...
    'Inputs','+-');
block(s,'VxCosPhi','simulink/Math Operations/Product',[375 285 410 320]);
block(s,'VySinPhi','simulink/Math Operations/Product',[375 365 410 400]);
block(s,'AlongWheelVelocity','simulink/Math Operations/Sum',[455 310 485 355], ...
    'Inputs','++');
block(s,'CasterRollRate','simulink/Math Operations/Gain',[535 310 620 345], ...
    'Gain','1/caster_radius_m');
outport(s,'phi',1,[760 155 790 175]);
outport(s,'phi_rate',2,[760 65 790 85]);
outport(s,'rolling_rate',3,[760 320 790 340]);
wire(s,'omega/1','OmegaTimesY/1'); wire(s,'v/1','HingeVx/1');
wire(s,'OmegaTimesY/1','HingeVx/2'); wire(s,'omega/1','HingeVy/1');
wire(s,'Phi/1','SinPhi/1'); wire(s,'Phi/1','CosPhi/1');
wire(s,'HingeVx/1','VxSinPhi/1'); wire(s,'SinPhi/1','VxSinPhi/2');
wire(s,'HingeVy/1','VyCosPhi/1'); wire(s,'CosPhi/1','VyCosPhi/2');
wire(s,'VxSinPhi/1','LateralHingeVelocity/1');
wire(s,'VyCosPhi/1','LateralHingeVelocity/2');
wire(s,'LateralHingeVelocity/1','AbsoluteCasterRate/1');
wire(s,'AbsoluteCasterRate/1','BodyRelativeRate/1');
wire(s,'omega/1','BodyRelativeRate/2'); wire(s,'BodyRelativeRate/1','Phi/1');
wire(s,'Phi/1','phi/1'); wire(s,'BodyRelativeRate/1','phi_rate/1');
wire(s,'HingeVx/1','VxCosPhi/1'); wire(s,'CosPhi/1','VxCosPhi/2');
wire(s,'HingeVy/1','VySinPhi/1'); wire(s,'SinPhi/1','VySinPhi/2');
wire(s,'VxCosPhi/1','AlongWheelVelocity/1'); wire(s,'VySinPhi/1','AlongWheelVelocity/2');
wire(s,'AlongWheelVelocity/1','CasterRollRate/1');
wire(s,'CasterRollRate/1','rolling_rate/1');
a = Simulink.Annotation(s,'phi is body-relative: subtract omega after the absolute caster rate.');
a.Position = [75 455];
end

function build_wheels(s)
inport(s,'v',1,[25 40 55 60]); inport(s,'omega',2,[25 145 55 165]);
block(s,'TurningSpeed','simulink/Math Operations/Gain',[100 140 180 170], ...
    'Gain','drive_track_m/2');
block(s,'LeftLinear','simulink/Math Operations/Sum',[235 35 265 75],'Inputs','+-');
block(s,'RightLinear','simulink/Math Operations/Sum',[235 185 265 225],'Inputs','++');
block(s,'LeftRate','simulink/Math Operations/Gain',[320 40 410 70],'Gain','1/drive_radius_m');
block(s,'RightRate','simulink/Math Operations/Gain',[320 190 410 220],'Gain','1/drive_radius_m');
block(s,'LR','simulink/Signal Routing/Mux',[470 80 475 185],'Inputs','2');
outport(s,'wheel_rates',1,[535 125 565 145]);
wire(s,'omega/1','TurningSpeed/1'); wire(s,'v/1','LeftLinear/1');
wire(s,'TurningSpeed/1','LeftLinear/2'); wire(s,'v/1','RightLinear/1');
wire(s,'TurningSpeed/1','RightLinear/2'); wire(s,'LeftLinear/1','LeftRate/1');
wire(s,'RightLinear/1','RightRate/1'); wire(s,'LeftRate/1','LR/1');
wire(s,'RightRate/1','LR/2'); wire(s,'LR/1','wheel_rates/1');
end

function blank(path,position)
add_block('built-in/Subsystem',path,'Position',position);
end
function inport(s,name,port,position)
add_block('built-in/Inport',[s '/' name],'Port',num2str(port),'Position',position);
end
function outport(s,name,port,position)
add_block('built-in/Outport',[s '/' name],'Port',num2str(port),'Position',position);
end
function block(s,name,source,position,varargin)
add_block(source,[s '/' name],'Position',position,varargin{:});
end
function wire(s,source,destination)
add_line(s,source,destination,'autorouting','on');
end
