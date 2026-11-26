function [JL,JR,JC,rH,rC,eD,nD,eC,nC] = m2_geometry(x,p)
%#codegen
% Cartesian velocity Jacobians, with world translation and body-relative swivel.
theta=x(3); psi=theta+x(4);
R=[cos(theta) -sin(theta);sin(theta) cos(theta)];
eD=R(:,1); nD=R(:,2); eC=[cos(psi);sin(psi)]; nC=[-sin(psi);cos(psi)];
rL=R*[-p(7);p(11)/2-p(8)]; rR=R*[-p(7);-p(11)/2-p(8)];
rH=R*[p(14)-p(7);p(15)-p(8)]; rC=rH-p(13)*eC;
JL=zeros(2,7); JR=zeros(2,7); JC=zeros(2,7);
JL(:,1:2)=eye(2); JR(:,1:2)=eye(2); JC(:,1:2)=eye(2);
JL(:,3)=[-rL(2);rL(1)]; JR(:,3)=[-rR(2);rR(1)];
JC(:,3)=[-rC(2);rC(1)]; JC(:,4)=-p(13)*nC;
end

