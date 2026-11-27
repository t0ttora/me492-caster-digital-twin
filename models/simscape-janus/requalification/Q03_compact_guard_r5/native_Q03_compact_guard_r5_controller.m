function native_Q03_compact_guard_r5_controller(block)
setup(block);
end
function setup(block)
block.NumDialogPrms=1;block.NumInputPorts=1;block.NumOutputPorts=2;
block.SetPreCompInpPortInfoToDynamic;block.SetPreCompOutPortInfoToDynamic;
block.InputPort(1).Dimensions=5;block.InputPort(1).DirectFeedthrough=false;
block.OutputPort(1).Dimensions=31;block.OutputPort(2).Dimensions=24;block.SampleTimes=[.01 0];block.SimStateCompliance='DefaultSimState';
block.RegBlockMethod('PostPropagationSetup',@post);block.RegBlockMethod('InitializeConditions',@init);block.RegBlockMethod('Outputs',@outputs);block.RegBlockMethod('Update',@update);
end
function post(block)
block.NumDworks=3;ns={'memory','command','observation_payload'};sz=[40 31 24];
for k=1:3,block.Dwork(k).Name=ns{k};block.Dwork(k).Dimensions=sz(k);block.Dwork(k).DatatypeID=0;block.Dwork(k).Complexity='Real';block.Dwork(k).UsedAsDiscState=true;end
end
function init(block)
s=zeros(40,1);s(3)=1;s(6)=-1;s(7)=-1;s(28)=-1;block.Dwork(1).Data=s;block.Dwork(2).Data=zeros(31,1);block.Dwork(3).Data=zeros(24,1);
end
function outputs(block)
block.OutputPort(1).Data=block.Dwork(2).Data;block.OutputPort(2).Data=block.Dwork(3).Data;
end
function update(block)
dt=.01;t=block.CurrentTime;mode=block.DialogPrm(1).Data;u=block.InputPort(1).Data;s=block.Dwork(1).Data;obs=block.Dwork(3).Data;
x=u(1);y=u(2);yaw=u(3);wl=u(4);wr=u(5);vmeas=((x-s(22))*cos(yaw)+(y-s(23))*sin(yaw))/dt;omegameas=wrap(yaw-s(21))/dt;s(21:23)=[yaw;x;y];
targetY=0;if mode==202,targetY=.15;elseif mode==203,targetY=-.15;end
gx=.5365;tx=.55;ty=targetY;ep=hypot(tx-x,ty-y);eh=wrap(-yaw);v=0;w=0;passed=0;phase=s(3);age=t-s(7);fresh=false;reason=0;
if t>=.5
 frameIndex=floor((t+1e-9)/.05);
 if frameIndex>s(6)
  s(6)=frameIndex;
  % Producer observation is injected BEFORE the consumer validates identity,
  % source timestamp, ordering and visibility. Logs retain the exact payload.
  o=produce_observation(t,x,y,yaw,targetY,mode,frameIndex);
  reason=0;
  if ~o.visible,reason=1;elseif o.id~=1,reason=3;elseif t-o.capture>.20+1e-9,reason=2;elseif o.capture<=s(28),reason=4;end
  accepted=reason==0;s(10)=accepted;s(29)=reason;s(11:18)=o.uv;s(26:27)=o.center;
  if accepted,s(7)=o.capture;s(8)=o.e;s(9)=o.depth;s(28)=o.capture;end
  obs=[o.capture;o.arrival;o.id;o.visible;accepted;reason;640;480;o.uv;o.center;o.e;o.depth;t;t-o.capture;double(o.visible);frameIndex];
 end
 reason=s(29);age=t-s(7);fresh=s(7)>=0&&age<=.20+1e-9;
 if ~fresh||~s(10)
  s(3)=1;s(4)=0;phase=4;
  if s(32)==0,s(32)=t;end
  if t-s(32)>=1,s(33)=1;end
 else
  s(32)=0;e=s(8);
  if s(33)>0,phase=9;reason=5;
  elseif s(3)==1
   w=clip(-1.8*e,.35);
   if abs(e)<=.02,s(4)=s(4)+dt;else,s(4)=0;end
   if s(4)>=.4
    s(4)=0;
    s(3)=2;
   end
  elseif s(3)==10
   % Differential-drive visual reverse arc on declared clear infinite floor.
   % The offset from marker bearing is bounded to .035 rad (~.0336 image
   % width fraction), inside the inherited .04 outer band for BOTH signs v.
   bearing=atan2(ty-y,.8-x);goalYaw=bearing-.035*s(30);w=clip(1.5*wrap(goalYaw-yaw),.25);
   if abs(wrap(goalYaw-yaw))<.015&&abs(vmeas)<.002&&abs(omegameas)<.03,s(4)=s(4)+dt;else,s(4)=0;end
   if s(4)>=.25,s(3)=11;s(4)=0;end
  elseif s(3)==11
   bearing=atan2(ty-y,.8-x);goalYaw=bearing-.035*s(30);w=clip(1.5*wrap(goalYaw-yaw),.25);v=-.06;
   z=.8-x;lateral=s(30)*(ty-y);
   % Short measured reverse stroke in a compact, explicitly clear fixture.
   if x<=.05,s(3)=12;s(4)=0;v=0;end
  elseif s(3)==12
   bearing=atan2(ty-y,.8-x);w=clip(1.5*wrap(bearing-.035*s(30)-yaw),.25);
   if abs(vmeas)<.002&&abs(omegameas)<.03,s(4)=s(4)+dt;else,s(4)=0;end
   if s(4)>=.4,s(3)=13;s(4)=0;end
  elseif s(3)==13
   w=clip(-1.8*e,.25);
   if abs(e)<.01&&abs(vmeas)<.002&&abs(omegameas)<.03,s(4)=s(4)+dt;else,s(4)=0;end
   if abs(e)<=.02&&abs(vmeas)<.002,s(35)=s(35)+dt;else,s(35)=0;end
   if s(35)>=.4,s(3)=14;s(4)=0;s(35)=0;end
  elseif s(3)==14
   bearing=atan2(ty-y,.8-x);goalYaw=bearing+.035*s(30);w=clip(1.5*wrap(goalYaw-yaw),.25);v=.06;
   if x>=.25,s(3)=15;s(4)=0;v=0;end
  elseif s(3)==15
   bearing=atan2(ty-y,.8-x);w=clip(1.5*wrap(bearing+.035*s(30)-yaw),.25);
   if abs(vmeas)<.002&&abs(omegameas)<.03,s(4)=s(4)+dt;else,s(4)=0;end
   if s(4)>=.4
    z=.8-x;lateral=s(30)*(ty-y);s(36)=s(36)+1;s(4)=0;
    if lateral/z<=.035*log(z/(.8-gx))-.018,s(31)=1;s(3)=1;else,s(3)=10;end
   end
  else
   % Gentle measured x/y/yaw trim. Lower speed near the optical stop.
   dx=gx-x;dy=ty-y;bearing=atan2(dy,.8-x);desiredYaw=atan2(2*dy,max(.10,abs(dx)));desiredYaw=bearing+clip(wrap(desiredYaw-bearing),.035);
   if abs(dx)<.004,desiredYaw=0;end
   w=clip(1.2*wrap(desiredYaw-yaw),.25);
   if dx>.0005,v=min(.06,.45*dx)*max(0,cos(yaw));elseif dx<-.003,v=max(-.012,.4*dx);end
   if abs(e)>.04,s(3)=1;s(4)=0;v=0;end
   good=s(9)<=.265&&abs(e)<=.02&&ep<=.02&&abs(eh)<=.05&&abs(vmeas)<=.02&&abs(omegameas)<=.05;
   if good,s(4)=s(4)+dt;else,s(4)=0;end
   if s(4)>=.25,s(3)=3;s(34)=1;end
   passed=double(s(34)>0);
   if s(31)==0&&abs(ty-y)>.008&&x>=.25,s(30)=sign(ty-y);s(3)=10;s(4)=0;v=0;end
  end
  if abs(e)>.04,v=0;end
 end
 if ~fresh||~s(10)||s(33)>0,v=0;w=0;end
else,phase=0;end
% Conservative supplemental inter-frame guard from the measured native pose.
% It can only inhibit motion; valid fresh camera/identity gating is still
% necessary. Official camera exit remains .04; this .037 margin is stricter.
if t>=.5&&fresh&&s(10)&&s(33)==0
 [predVisible,predError]=camera(x,y,yaw,targetY);
 if ~predVisible,v=0;w=0;reason=7;elseif abs(predError)>.037,v=0;reason=6;end
end
desired=[(v-.19*w)/.075;(v+.19*w)/.075];errors=desired-[wl;wr];tau=zeros(2,1);
for j=1:2,trial=s(j)+errors(j)*dt;raw=.25*errors(j)+.08*trial;tau(j)=clip(raw,.25);if abs(raw)<=.25||sign(raw)~=sign(errors(j)),s(j)=trial;end;end
if t<.5,tau(:)=0;s(1:2)=0;end
if t>=.5&&phase~=4&&phase~=9,phase=s(3);end
if s(33)>0,phase=9;reason=5;end
o=[tau;v;w;phase;tx;ty;eh;ep;s(8);s(9);s(10);fresh;age;s(4);passed;s(11:18);s(7);obs(1);s(26);s(27);reason;vmeas;omegameas];
block.Dwork(1).Data=s;block.Dwork(2).Data=o;block.Dwork(3).Data=obs;
end
function o=produce_observation(t,x,y,yaw,targetY,mode,frameIndex)
[visible,e,depth,uv,center]=camera(x,y,yaw,targetY);
o=struct('capture',t,'arrival',t,'id',1,'visible',visible,'e',e,'depth',depth,'uv',uv,'center',center);
if mode==206&&t>=2&&t<2.6,o.visible=false;end
if mode==216&&t>=2,o.visible=false;end
if mode==208,o.id=2;end
if mode==213&&t>=2,o.capture=t-.30;end
if mode==214&&t>=2,o.capture=1.95;end
if mode==220&&t>=1&&t<3
 % Synthetic optical perturbation: rigid horizontal pixel displacement of
 % the entire quadrilateral and physical center, consumed at runtime.
 shift=640*.065*sin(2*pi*1.25*(t-1));o.uv(1:2:8)=o.uv(1:2:8)+shift;o.center(1)=o.center(1)+shift;o.e=(o.center(1)-320)/640;
 o.visible=o.visible&&all(o.uv(1:2:8)>=0&o.uv(1:2:8)<=640);
end
end
function [visible,e,depth,uv,center]=camera(x,y,yaw,targetY)
dy=[.05 -.05 -.05 .05];dz=[.05 .05 -.05 -.05];px=.8-x;py=targetY+dy-y;
optz=cos(yaw)*px+sin(yaw)*py;optx=sin(yaw)*px-cos(yaw)*py;opty=-dz;
uu=320+500*optx./optz;vv=240+500*opty./optz;uv=reshape([uu;vv],8,1);
depth=cos(yaw)*px+sin(yaw)*(targetY-y);center=[320+500*(sin(yaw)*px-cos(yaw)*(targetY-y))/depth;240];e=(center(1)-320)/640;
area=abs(sum(uu.*vv([2 3 4 1])-vv.*uu([2 3 4 1])))/2;d1=[uu(3)-uu(1);vv(3)-vv(1)];d2=[uu(4)-uu(2);vv(4)-vv(2)];diagDet=d1(1)*d2(2)-d1(2)*d2(1);
visible=all(isfinite([uv;center]))&&depth>.02&&all(optz>.02)&&all(uu>=0&uu<=640&vv>=0&vv<=480)&&area>1e-6&&abs(diagDet)>1e-9;
end
function a=wrap(a),a=atan2(sin(a),cos(a));end
function a=clip(a,b),a=min(b,max(-b,a));end
