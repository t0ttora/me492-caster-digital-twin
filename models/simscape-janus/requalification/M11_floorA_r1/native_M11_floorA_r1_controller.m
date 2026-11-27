function native_M11_floorA_r1_controller(block)
% Discrete measured-pose controller for native Simscape experiments.
setup(block);
end
function setup(block)
block.NumDialogPrms=1;block.NumInputPorts=1;block.NumOutputPorts=1;
block.SetPreCompInpPortInfoToDynamic;block.SetPreCompOutPortInfoToDynamic;
block.InputPort(1).Dimensions=5;block.InputPort(1).DirectFeedthrough=false;
block.OutputPort(1).Dimensions=31;block.SampleTimes=[.01 0];block.SimStateCompliance='DefaultSimState';
block.RegBlockMethod('PostPropagationSetup',@post);block.RegBlockMethod('InitializeConditions',@init);block.RegBlockMethod('Outputs',@outputs);block.RegBlockMethod('Update',@update);
end
function post(block)
block.NumDworks=2;
block.Dwork(1).Name='memory';block.Dwork(1).Dimensions=27;block.Dwork(1).DatatypeID=0;block.Dwork(1).Complexity='Real';block.Dwork(1).UsedAsDiscState=true;
block.Dwork(2).Name='command';block.Dwork(2).Dimensions=31;block.Dwork(2).DatatypeID=0;block.Dwork(2).Complexity='Real';block.Dwork(2).UsedAsDiscState=true;
end
function init(block)
s=zeros(27,1);s(3)=1;s(6)=-1;s(7)=-1;block.Dwork(1).Data=s;block.Dwork(2).Data=zeros(31,1);
end
function outputs(block),block.OutputPort(1).Data=block.Dwork(2).Data;end
function update(block)
dt=.01;t=block.CurrentTime;mode=block.DialogPrm(1).Data;u=block.InputPort(1).Data;
x=u(1);y=u(2);yaw=u(3);wl=u(4);wr=u(5);s=block.Dwork(1).Data;
v=0;w=0;tx=x;ty=y;eh=0;ep=0;phase=s(3);passed=0;fresh=0;age=0;
vmeas=((x-s(22))*cos(yaw)+(y-s(23))*sin(yaw))/dt;omegameas=wrap(yaw-s(21))/dt;
s(21:23)=[yaw;x;y];reason=0;
if t>=.5
 if mode>=100
  tx=0;ty=0;eh=wrap(-yaw);ep=hypot(x,y);
  if mode==102
   if t<4,v=min(.04,.04*(t-.5));w=clip(-1.5*yaw-2*y,.25);s(3)=1;else,s(3)=2;w=clip(-1.5*yaw,.2);end
   if t>=4&&abs(vmeas)<=.002&&abs(omegameas)<=.01,s(4)=s(4)+dt;else,s(4)=0;end
   if s(4)>=.4,passed=1;s(3)=3;end
  elseif mode==103
   tx=.4;ty=.4;ep=hypot(tx-x,ty-y);eh=wrap(pi/2-yaw);
   if phase==1
    theta=max(0,min(pi/2,atan2(x,.4-y)));radial=hypot(x,.4-y)-.4;
    v=min(.04,.5*ep);w=clip(v/.4+1.5*wrap(theta-yaw)+.8*radial,.3);
    if ep<.012,s(3)=2;s(4)=0;end
   else
    [v,w,s(19)]=pose_hold(x,y,yaw,tx,ty,pi/2,s(19));
    if ep<=.02&&abs(eh)<=.05&&abs(vmeas)<=.02&&abs(omegameas)<=.05,s(4)=s(4)+dt;else,s(4)=0;end
    if s(4)>=.25,s(3)=3;passed=1;end
   end
  elseif mode==109||mode==110
   tx=.15;ty=0;ep=hypot(tx-x,ty-y);eh=wrap(pi/2-yaw);
   if mode==110,tx=0;ep=hypot(x,y);end
   if phase==1
    if mode==109,v=min(.04,.04*(t-.5));w=clip(-1.5*yaw-2*y,.25);end
    if x>=.15||mode==110,s(3)=2;s(4)=0;s(5)=x;s(20)=y;end
   elseif phase==2
    if abs(vmeas)<=.002&&abs(omegameas)<=.01,s(4)=s(4)+dt;else,s(4)=0;end
    if s(4)>=.4,s(3)=3;s(4)=0;end
   else
    w=clip(1.2*eh,.3);
    if abs(eh)<=.05&&abs(vmeas)<=.02&&abs(omegameas)<=.05,s(4)=s(4)+dt;else,s(4)=0;end
    if s(4)>=.25,s(3)=4;passed=1;end
   end
  elseif mode==111||mode==112
   tx=.65;ty=0;ep=hypot(tx-x,ty-y);eh=wrap(-yaw);
   if phase==1,v=min(.06,.04*(t-.5));w=clip(-1.5*yaw-2*y,.25);if x>=.65,s(3)=2;s(4)=0;end
   else,w=clip(-1.5*yaw,.25);if abs(vmeas)<=.002&&abs(omegameas)<=.01,s(4)=s(4)+dt;else,s(4)=0;end;if s(4)>=.4,s(3)=3;passed=1;end;end
  elseif mode==108
   if phase==1
    v=min(.04,.04*(t-.5));w=clip(-1.5*yaw-2*y,.25);
    if x>=.15,s(3)=2;s(4)=0;end
   elseif phase==2
    w=clip(-1.5*yaw,.2);
    if abs(vmeas)<=.002&&abs(omegameas)<=.01,s(4)=s(4)+dt;else,s(4)=0;end
    if s(4)>=.4,s(3)=3;s(4)=0;s(5)=x;s(19)=t;end
   elseif phase==3
    v=-min(.04,.04*(t-s(19)));w=clip(-1.5*yaw+2*y,.25);
    if s(5)-x>=.15,s(3)=4;s(4)=0;end
   else
    w=clip(-1.5*yaw,.2);
    if abs(vmeas)<=.002&&abs(omegameas)<=.01,s(4)=s(4)+dt;else,s(4)=0;end
    if s(4)>=.4,passed=1;s(3)=5;end
   end
  end
 elseif mode==1||mode==8
  gx=1;gy=0;
  if mode==1
   tx=min(1,max(s(5),x)+.08);ty=.10*sin(2*pi*tx)^3;s(5)=max(s(5),min(1,x));
  else
   R=.3/(pi/3);gx=2*R*sin(pi/3);gy=2*R*(1-cos(pi/3));q=linspace(s(5),2,301);aa=(pi/3)*min(q,2-q);
   xx=R*sin(aa);yy=R*(1-cos(aa));back=q>1;xx(back)=2*R*sin(pi/3)-R*sin(aa(back));yy(back)=R*(1-2*cos(pi/3)+cos(aa(back)));
   [~,near]=min((xx-x).^2+(yy-y).^2);s(5)=q(near);look=min(2,s(5)+.2);aa=(pi/3)*min(look,2-look);
   tx=R*sin(aa);ty=R*(1-cos(aa));if look>1,tx=2*R*sin(pi/3)-R*sin(aa);ty=R*(1-2*cos(pi/3)+cos(aa));end
  end
  ep=hypot(gx-x,gy-y);
  if phase==1
   eh=wrap(atan2(ty-y,tx-x)-yaw);v=min(.08,.7*ep)*max(0,cos(eh));w=clip(2*v*sin(eh)/max(.05,hypot(tx-x,ty-y)),.65);
   if ep<=.018,s(3)=2;s(4)=0;end
  elseif phase==2||phase==3
   eh=wrap(-yaw);[v,w,s(19)]=pose_hold(x,y,yaw,gx,gy,0,s(19));
   if ep<=.02&&abs(eh)<=.05&&abs(vmeas)<=.02&&abs(omegameas)<=.05,s(4)=s(4)+dt;else,s(4)=0;end
   if s(4)>=.25,s(3)=3;end
   if phase==3,passed=1;end
  end
 elseif mode==2||mode==7
  pts=[.5 0;.5 0;.5 -.5;.5 -.5;0 -.5;0 -.5;0 0;0 0];heads=[0 -pi/2 -pi/2 pi pi pi/2 pi/2 0];
  if mode==7,pts(:,2)=-pts(:,2);heads=-heads;end
  if phase<=8
   tx=pts(phase,1);ty=pts(phase,2);ep=hypot(tx-x,ty-y);eh=wrap(heads(phase)-yaw);
   if mod(phase,2)==1
    along=(tx-x)*cos(heads(phase))+(ty-y)*sin(heads(phase));cross=-(tx-x)*sin(heads(phase))+(ty-y)*cos(heads(phase));
    desired=heads(phase)+atan2(2*cross,max(.08,abs(along)));err=wrap(desired-yaw);
    v=min(.08,max(0,.8*along))*max(0,cos(err));w=clip(2*err,.65);
    if ep<.008,v=0;w=clip(2*eh,.65);end
    good=ep<=.02&&abs(eh)<=.06&&abs(vmeas)<=.02;
   else
    w=clip(2*eh,.65);good=ep<=.05&&abs(eh)<=5*pi/180&&abs(vmeas)<=.02&&abs(omegameas)<=.05;
    if phase==8,[v,w,s(19)]=pose_hold(x,y,yaw,0,0,0,s(19));good=ep<=.02&&abs(eh)<=.05&&abs(vmeas)<=.02&&abs(omegameas)<=.05;end
   end
   if good,s(4)=s(4)+dt;else,s(4)=0;end
   if s(4)>=.4,s(3)=phase+1;s(4)=0;end
  else,[v,w,s(19)]=pose_hold(x,y,yaw,0,0,0,s(19));end
  passed=min(8,s(3)-1);
 else
  % Known marker plane. Only simulated measured pose enters projection.
  targetY=0;if mode==4,targetY=.15;elseif mode==5,targetY=-.15;end
  tx=.8;ty=targetY;ep=hypot(tx-x,ty-y);
  frameIndex=floor((t+1e-9)/.05);
  if frameIndex>s(6)
   s(6)=frameIndex;[visible,e,depth,uv,center]=camera(x,y,yaw,targetY);
   if mode==6&&t>=1&&t<1.6,visible=false;end
   s(10)=visible;s(11:18)=uv;s(26:27)=center;
   if visible,s(7)=t;s(8)=e;s(9)=depth;end
  end
  age=t-s(7);fresh=s(7)>=0&&age<=.20+1e-9;
  if ~fresh||~s(10)
   s(3)=1;s(4)=0;phase=4;reason=1;if ~fresh,reason=2;end
  else
   e=s(8);w=clip(-1.8*e,.6);
   if s(3)==1
    if abs(e)<=.02,s(4)=s(4)+dt;else,s(4)=0;end
    if s(4)>=.4,s(3)=2;s(4)=0;end
   elseif s(3)==2
    if abs(e)>.04,s(3)=1;s(4)=0;
    else,v=min(.06,max(0,.6*(s(9)-.25)));end
    if s(9)<=.265&&abs(e)<=.02&&abs(vmeas)<=.02,s(4)=s(4)+dt;else,s(4)=0;end
    if s(4)>=.4,s(3)=3;end
   else,v=0;w=0;passed=1;end
   if abs(e)>.04,v=0;end
  end
 end
end
if t<.5,v=0;w=0;phase=0;end
if (mode==1||mode==8)&&s(3)==3,passed=1;end
if (mode==2||mode==7)&&s(3)==9,passed=8;end
if mode>=3&&mode<=6&&s(3)==3,v=0;w=0;passed=1;end
% PI wheel-speed servo with conditional anti-windup.
desired=[(v-.19*w)/.075;(v+.19*w)/.075];errors=desired-[wl;wr];tau=zeros(2,1);
for j=1:2
 trial=s(j)+errors(j)*dt;raw=.25*errors(j)+.08*trial;tau(j)=clip(raw,.25);
 if abs(raw)<=.25||sign(raw)~=sign(errors(j)),s(j)=trial;end
end
if t<.5,tau(:)=0;s(1:2)=0;end
if phase~=4&&t>=.5,phase=s(3);end
o=[tau;v;w;phase;tx;ty;eh;ep;s(8);s(9);s(10);fresh;age;s(4);passed;s(11:18);s(7);s(6)*.05;s(26);s(27);reason;vmeas;omegameas];
block.Dwork(1).Data=s;block.Dwork(2).Data=o;
end
function [visible,e,depth,uv,center]=camera(x,y,yaw,targetY)
% Physical marker TL,TR,BR,BL order when upright, viewed from the robot.
dy=[.05 -.05 -.05 .05];dz=[.05 .05 -.05 -.05];px=.8-x;py=targetY+dy-y;
optz=cos(yaw)*px+sin(yaw)*py;optx=sin(yaw)*px-cos(yaw)*py;opty=-dz;
uu=320+500*optx./optz;vv=240+500*opty./optz;uv=reshape([uu;vv],8,1);
depth=cos(yaw)*px+sin(yaw)*(targetY-y);
% Project physical center; under perspective this is the diagonal intersection,
% not the arithmetic mean of four image corners. Nominal center optical y=0.
center=[320+500*(sin(yaw)*px-cos(yaw)*(targetY-y))/depth;240];
e=(center(1)-320)/640;
area=abs(sum(uu.*vv([2 3 4 1])-vv.*uu([2 3 4 1])))/2;
d1=[uu(3)-uu(1);vv(3)-vv(1)];d2=[uu(4)-uu(2);vv(4)-vv(2)];
diagDet=d1(1)*d2(2)-d1(2)*d2(1);
visible=all(isfinite([uv;center]))&&depth>.02&&all(optz>.02)&&...
 all(uu>=0&uu<=640&vv>=0&vv<=480)&&area>1e-6&&abs(diagDet)>1e-9;
end
function a=wrap(a),a=atan2(sin(a),cos(a));end
function a=clip(a,b),a=min(b,max(-b,a));end
function [v,w,aligned]=pose_hold(x,y,yaw,gx,gy,gh,aligned)
% Hybrid measured-pose correction, followed by heading hold. Final latching
% does not disable feedback. Re-entry after drift is explicit and logged.
dx=gx-x;dy=gy-y;dist=hypot(dx,dy);
if aligned&&dist>.015,aligned=0;end
if ~aligned&&dist<=.007,aligned=1;end
if aligned,v=0;w=clip(1.2*wrap(gh-yaw),.35);
else
 alpha=wrap(atan2(dy,dx)-yaw);direction=1;
 if abs(alpha)>pi/2,alpha=wrap(alpha+pi);direction=-1;end
 v=direction*min(.035,.6*dist)*max(0,cos(alpha));w=clip(1.2*alpha,.35);
end
end
