function [w,c] = m2_contact(x,p,casterEnabled)
%#codegen
% World contact forces, regularized rolling/bearing/bore torques, and slip data.
nu=x(5:11);
[JL,JR,JC,~,~,eD,nD,eC,nC]=m2_geometry(x,p);
casterX=p(14)-p(13)*cos(x(4)); casterY=p(15)-p(13)*sin(x(4));
Nc=p(16)*(p(1)*p(7)+p(2)*casterX)/casterX;
normalDifference=2*p(16)*(p(1)*p(8)+p(2)*casterY)/p(11)-2*Nc*casterY/p(11);
Nl=0.5*((p(1)+p(2))*p(16)-Nc+normalDifference);
Nr=0.5*((p(1)+p(2))*p(16)-Nc-normalDifference);
assert(Nl>0 && Nr>0 && Nc>0);
vL=JL*nu; vR=JR*nu; vC=JC*nu;
sL=[eD.'*vL-p(10)*nu(5);nD.'*vL];
sR=[eD.'*vR-p(10)*nu(6);nD.'*vR];
sC=[eC.'*vC-p(12)*nu(7);nC.'*vC];
fL=passive_friction(sL,p(19),p(20),p(17)*Nl);
fR=passive_friction(sR,p(19),p(20),p(17)*Nr);
[fC,bore]=passive_caster_contact(sC,p(21),p(22),p(18)*Nc, ...
    nu(3)+nu(4),p(31),p(32));
rrL=p(23)*Nl*p(10)*tanh(nu(5)/p(29))+p(25)*nu(5);
rrR=p(23)*Nr*p(10)*tanh(nu(6)/p(29))+p(25)*nu(6);
rrC=p(24)*Nc*p(12)*tanh(nu(7)/p(29))+p(26)*nu(7);
bearing=p(27)*nu(4)+p(28)*tanh(nu(4)/p(30));
if casterEnabled<0.5
    fC=zeros(2,1); rrC=0; bore=0;
end

function [f,bore]=passive_caster_contact(s,kx,ky,limit,absoluteRate,effectiveRadius,rateScale)
% Phenomenological combined translation/torsion friction ellipsoid.
% norm([f; bore/effectiveRadius]) <= mu*N. Bore is a resisting torque magnitude.
rawSpin=0;
if effectiveRadius>0, rawSpin=limit*absoluteRate/rateScale; end
raw=[kx*s(1);ky*s(2);rawSpin];
if limit<=0
    f=zeros(2,1); bore=0;
else
    scale=limit/sqrt(limit^2+raw.'*raw);
    f=-scale*raw(1:2); bore=scale*raw(3)*effectiveRadius;
end
end
FL=[eD nD]*fL; FR=[eD nD]*fR; FC=[eC nC]*fC;
w=[FL;FR;FC;rrL;rrR;rrC;bearing;bore];
c=[sL;sR;sC;Nl;Nr;Nc;fL;fR;fC];
end

function f=passive_friction(s,kx,ky,limit)
% Saturated combined-slip law: f dot s <= 0 and norm(f) <= mu*N.
raw=[kx*s(1);ky*s(2)];
if limit<=0, f=zeros(2,1); else, f=-raw*limit/sqrt(limit^2+raw.'*raw); end
end
