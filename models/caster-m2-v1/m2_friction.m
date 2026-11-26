function w=m2_friction(x,p,slip,normal,casterEnabled)
%#codegen
% Passive contact: identical assumptions to preserved M2 v0.
nu=x(5:11);[~,~,~,~,~,eD,nD,eC,nC]=m2_geometry(x,p);
fL=passive_friction(slip(1:2),p(19),p(20),p(17)*normal(1));
fR=passive_friction(slip(3:4),p(19),p(20),p(17)*normal(2));
[fC,bore]=caster_patch(slip(5:6),p(21),p(22),p(18)*normal(3),nu(3)+nu(4),p(31),p(32));
rrL=p(23)*normal(1)*p(10)*tanh(nu(5)/p(29))+p(25)*nu(5);
rrR=p(23)*normal(2)*p(10)*tanh(nu(6)/p(29))+p(25)*nu(6);
rrC=p(24)*normal(3)*p(12)*tanh(nu(7)/p(29))+p(26)*nu(7);
bearing=p(27)*nu(4)+p(28)*tanh(nu(4)/p(30));
if casterEnabled<0.5,fC=zeros(2,1);rrC=0;bore=0;end
w=[[eD nD]*fL;[eD nD]*fR;[eC nC]*fC;rrL;rrR;rrC;bearing;bore];
end
function f=passive_friction(s,kx,ky,limit)
raw=[kx*s(1);ky*s(2)];
if limit<=0,f=zeros(2,1);else,f=-raw*limit/sqrt(limit^2+raw.'*raw);end
end
function [f,bore]=caster_patch(s,kx,ky,limit,absoluteRate,radius,rateScale)
rawSpin=0;if radius>0,rawSpin=limit*absoluteRate/rateScale;end
raw=[kx*s(1);ky*s(2);rawSpin];
if limit<=0,f=zeros(2,1);bore=0;else
scale=limit/sqrt(limit^2+raw.'*raw);f=-scale*raw(1:2);bore=scale*raw(3)*radius;
end
end
