function path=build_m2()
% Extend a preserved existing M2 model; never overwrite the user's M0/M2 v0.
root=fileparts(mfilename('fullpath'));addpath(root);
model='Me492CasterM2V1';path=fullfile(root,'models',[model '.slx']);
assert(~isfile(path),'ME492:PreserveRevision','Existing revised SLX must be preserved before rebuilding.');
assert(~bdIsLoaded(model),'ME492:RevisionLoaded','Close the revision before rebuilding.');
source=fullfile(root,'models','Me492CasterM2.slx');assert(isfile(source));
% A unique working copy avoids loading or renaming an already-open v0 model.
[~,suffix]=fileparts(tempname(fullfile(root,'models')));temporaryModel=['M2Build_' suffix];
temporaryPath=fullfile(root,'models',[temporaryModel '.slx']);copyfile(source,temporaryPath);
cleanup=onCleanup(@() clean_build_copy(temporaryModel,temporaryPath));
load_system(temporaryPath);save_system(temporaryModel,path,'AllowPrompt',false);
% Saving under a new name changes the loaded diagram name, not the preserved file.
set_param(model,'InitFcn',sprintf(['addpath(fileparts(fileparts(get_param(bdroot,''FileName''))));' ...
 ' m2_model_init(bdroot);']),'ScreenColor','white','ZoomFactor','100');
lineHandles=find_system(model,'FindAll','on','SearchDepth',1,'Type','line');
parents=zeros(size(lineHandles));for k=1:numel(lineHandles),parents(k)=get_param(lineHandles(k),'LineParent');end
for h=lineHandles(parents==-1).',delete_line(h);end
for name={'WheelGroundForces','BodyCasterInertiaDynamics'},delete_block([model '/' name{1}]);end
contacts([model '/WheelGroundForces']);inertia([model '/BodyCasterInertiaDynamics']);
set_param([model '/IndependentWheelTorques'],'Position',[35 105 220 155]);
set_param([model '/PhysicalParameters'],'Position',[35 330 220 375]);
set_param([model '/CasterGroundContactEnabled'],'Position',[35 440 220 485]);
set_param([model '/WheelGroundForces'],'Position',[355 305 655 490]);
set_param([model '/BodyCasterInertiaDynamics'],'Position',[715 90 1030 265]);
set_param([model '/MechanicalState'],'Position',[1115 130 1175 205]);
set_param([model '/StateRecord'],'Position',[1300 115 1460 160]);
set_param([model '/EnergyPowerRecord'],'Position',[1300 230 1460 275]);
set_param([model '/ForceRecord'],'Position',[795 355 950 400]);
set_param([model '/ContactRecord'],'Position',[795 455 950 500]);
wire(model,'IndependentWheelTorques/1','BodyCasterInertiaDynamics/2','tauL, tauR [N m]');
wire(model,'PhysicalParameters/1','WheelGroundForces/2','p32: SI parameters');
wire(model,'PhysicalParameters/1','BodyCasterInertiaDynamics/3');
wire(model,'CasterGroundContactEnabled/1','WheelGroundForces/3','tangential contact flag');
wire(model,'WheelGroundForces/1','BodyCasterInertiaDynamics/4','ground forces + resistance');
wire(model,'BodyCasterInertiaDynamics/1','MechanicalState/1','d/dt: 11 physics + 2 energy');
wire(model,'MechanicalState/1','StateRecord/1','x13 [m, rad, m/s, rad/s, J]');
wire(model,'BodyCasterInertiaDynamics/2','EnergyPowerRecord/1','T, Pin, Pdiss, Mc, yaw acceleration');
wire(model,'WheelGroundForces/1','ForceRecord/1');wire(model,'WheelGroundForces/2','ContactRecord/1');
% A visible lower feedback route, with a common branch to both physical stages.
h=add_line(model,'MechanicalState/1','WheelGroundForces/1','autorouting','on');
set_param(h,'Name','state feedback x13');
wire(model,'MechanicalState/1','BodyCasterInertiaDynamics/1');
old=find_system(model,'FindAll','on','SearchDepth',1,'Type','annotation');
for k=1:numel(old),delete(get_param(old(k),'Object'));end
note(model,'M2 v1 | TORQUE -> CONTACT -> COUPLED DYNAMICS -> STATE',[35 20],18);
note(model,sprintf(['11 physical states + Wmotor and Dloss = 13 integrated states\n' ...
 'World: +X, +Y; body: +x forward, +y left; +yaw CCW\n' ...
 'B is body COM; A is drive-axle ground origin. Caster: psi = theta + phi.']),[35 555],14);
note(model,sprintf(['All mass, COM, inertia and friction values are SYNTHETIC.\n' ...
 'Quasi-static vertical loads. No impact, tipping, shimmy or robot validation.\n' ...
 'Double-click contacts / inertia for executable physical stages.']),[740 555],14);
style(model);polish_m2(model);save_system(model,path);export_m2_visuals(model);
fprintf('Revised M2 saved: %s\n',path);
end
function clean_build_copy(model,file)
if bdIsLoaded(model),close_system(model,0);end
if isfile(file),delete(file);end
end
function contacts(sys)
add_block('built-in/Subsystem',sys);port(sys,'x13',1,[25 105 55 125],'Inport');
port(sys,'p32',2,[25 275 55 295],'Inport');port(sys,'enabled',3,[25 445 55 465],'Inport');
port(sys,'w11',1,[1205 160 1235 180],'Outport');port(sys,'c15',2,[1205 420 1235 440],'Outport');
fn(sys,'SupportLoadBalance',[155 30 385 135], ...
 'function N=loads(x,p)\n%#codegen\nN=m2_support_loads(x,p);\nend\n');
fn(sys,'WheelSlipVelocities',[155 195 385 300], ...
 'function s=slip(x,p)\n%#codegen\ns=m2_slip(x,p);\nend\n');
fn(sys,'FrictionAndResistances',[500 75 770 275], ...
 'function w=forces(x,p,s,N,enabled)\n%#codegen\nw=m2_friction(x,p,s,N,enabled);\nend\n');
fn(sys,'ContactDiagnostics',[865 340 1110 505], ...
 'function c=diagnostics(x,p,s,N,w)\n%#codegen\nc=m2_contact_diagnostics(x,p,s,N,w);\nend\n');
for name={'SupportLoadBalance','WheelSlipVelocities','FrictionAndResistances','ContactDiagnostics'}
 wire(sys,'x13/1',[name{1} '/1']);wire(sys,'p32/1',[name{1} '/2']);end
wire(sys,'WheelSlipVelocities/1','FrictionAndResistances/3','sL, sR, sC [m/s]');
wire(sys,'SupportLoadBalance/1','FrictionAndResistances/4','NL, NR, NC [N]');
wire(sys,'enabled/1','FrictionAndResistances/5');
wire(sys,'WheelSlipVelocities/1','ContactDiagnostics/3');
wire(sys,'SupportLoadBalance/1','ContactDiagnostics/4');
wire(sys,'FrictionAndResistances/1','ContactDiagnostics/5');
wire(sys,'FrictionAndResistances/1','w11/1','world F [N], resistance tau [N m]');
wire(sys,'ContactDiagnostics/1','c15/1');
note(sys,'CONTACTS | SUPPORT LOADS -> SLIP -> PASSIVE FRICTION',[25 -35],16);
note(sys,sprintf(['sum N = (mb+mc) g; support x/y moments balance. No dynamic load transfer.\n' ...
 'sLong = e'' vCenter - radius*omega; sLat = n'' vCenter.\n' ...
 'Caster force and bore torque share one friction budget. Bearing uses phiDot; bore uses thetaDot+phiDot.']),[25 570],13);
end
function inertia(sys)
add_block('built-in/Subsystem',sys);
for k=1:4,names={'x13','tau2','p32','w11'};port(sys,names{k},k,[20 75+110*(k-1) 50 95+110*(k-1)],'Inport');end
port(sys,'dx13',1,[1085 150 1115 170],'Outport');port(sys,'energy5',2,[1085 440 1115 460],'Outport');
fn(sys,'MassMatrixAndBias',[170 30 425 155], ...
 'function [M,b]=inertia(x,p)\n%#codegen\n[M,b]=m2_mass_terms(x,p);\nend\n');
fn(sys,'GroundAndDriveForces',[170 235 425 395], ...
 'function Q=generalized(x,tau,p,w)\n%#codegen\nQ=m2_generalized_forces(x,tau(:),p,w);\nend\n');
fn(sys,'CoupledAcceleration',[560 65 805 245], ...
 'function dx11=motion(x,M,b,Q)\n%#codegen\ndx11=m2_motion_rhs(x,M,b,Q);\nend\n');
fn(sys,'PowerAndEnergy',[560 345 805 530], ...
 'function [rates,o]=power(x,tau,p,w,dx11)\n%#codegen\n[rates,o]=m2_energy_power(x,tau(:),p,w,dx11);\nend\n');
add_block('simulink/Signal Routing/Mux',[sys '/PhysicsAndAccounting'],'Inputs','2','Position',[950 105 965 225]);
wire(sys,'x13/1','MassMatrixAndBias/1');wire(sys,'p32/1','MassMatrixAndBias/2');
for name={'GroundAndDriveForces','PowerAndEnergy'}
 for k=1:4,names={'x13','tau2','p32','w11'};wire(sys,[names{k} '/1'],sprintf('%s/%d',name{1},k));end,end
wire(sys,'x13/1','CoupledAcceleration/1');
wire(sys,'MassMatrixAndBias/1','CoupledAcceleration/2','M [7 x 7]');
wire(sys,'MassMatrixAndBias/2','CoupledAcceleration/3','centripetal bias');
wire(sys,'GroundAndDriveForces/1','CoupledAcceleration/4','Q [7]');
wire(sys,'CoupledAcceleration/1','PowerAndEnergy/5');
wire(sys,'CoupledAcceleration/1','PhysicsAndAccounting/1','11 physical derivatives');
wire(sys,'PowerAndEnergy/1','PhysicsAndAccounting/2','Wdot=Pin; Ddot=Pdiss');
wire(sys,'PhysicsAndAccounting/1','dx13/1');wire(sys,'PowerAndEnergy/2','energy5/1');
note(sys,'INERTIA | M * nuDot + bias = Q',[20 -35],16);
note(sys,sprintf(['Caster translational mass and absolute swivel inertia couple body and caster motion.\n' ...
 'Bearing: Qphi -= tauBearing. Bore: Qtheta and Qphi -= tauBore.\n' ...
 'Power balance: T - T0 + D - W = 0. Energy accounting does not drive motion.']),[20 600],13);
end
function port(sys,name,index,pos,kind)
add_block(['built-in/' kind],[sys '/' name],'Port',num2str(index),'Position',pos);
end
function fn(sys,name,pos,script)
path=[sys '/' name];add_block('simulink/User-Defined Functions/MATLAB Function',path,'Position',pos);
charts=sfroot;c=charts.find('-isa','Stateflow.EMChart','Path',path);c.Script=strrep(script,'\n',newline);
set_param(path,'BackgroundColor','[0.90 0.95 0.98]');
end
function wire(sys,source,destination,name)
h=add_line(sys,source,destination,'autorouting','on');if nargin==4,set_param(h,'Name',name);end
end
function note(sys,text,pos,size)
a=Simulink.Annotation(sys,text);a.Position=pos;a.FontSize=size;a.FontName='Arial';a.ForegroundColor='black';
end
function style(model)
blocks=find_system(model,'Type','block');for k=1:numel(blocks)
set_param(blocks{k},'FontSize','13','FontName','Arial','ForegroundColor','black');end
set_param([model '/WheelGroundForces'],'BackgroundColor','[0.90 0.95 0.98]');
set_param([model '/BodyCasterInertiaDynamics'],'BackgroundColor','[0.90 0.95 0.98]');
set_param([model '/MechanicalState'],'BackgroundColor','[0.95 0.95 0.95]');
end
