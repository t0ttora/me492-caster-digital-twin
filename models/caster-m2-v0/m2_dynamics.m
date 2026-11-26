function [dx,o] = m2_dynamics(x,tau,p,w)
%#codegen
% Generalized Newton-Euler/Lagrange balance: M*nu_dot + bias = Q.
nu=x(5:11);
[JL,JR,JC,~,rC,eD,~,eC,~]=m2_geometry(x,p);
GL=JL; GR=JR; GC=JC;
GL(:,5)=-p(10)*eD; GR(:,6)=-p(10)*eD; GC(:,7)=-p(12)*eC;
FL=w(1:2); FR=w(3:4); FC=w(5:6);
Q=GL.'*FL+GR.'*FR+GC.'*FC;
Q(5)=Q(5)+tau(1)-w(7); Q(6)=Q(6)+tau(2)-w(8); Q(7)=Q(7)-w(9);
% Bearing friction acts on relative swivel phi. Ground bore acts on theta+phi.
Q(4)=Q(4)-w(10); Q(3:4)=Q(3:4)-w(11);
[M,bias]=m2_mass_terms(x,p);
acceleration=M\(Q-bias);
VC=JC*nu;
energy=0.5*p(1)*(nu(1:2).'*nu(1:2))+0.5*p(2)*(VC.'*VC) ...
      +0.5*p(3)*nu(3)^2+0.5*p(4)*(nu(3)+nu(4))^2 ...
      +0.5*p(5)*(nu(5)^2+nu(6)^2)+0.5*p(6)*nu(7)^2;
motorPower=tau.'*nu(5:6);
dissipation=-FL.'*(GL*nu)-FR.'*(GR*nu)-FC.'*(GC*nu) ...
    +w(7)*nu(5)+w(8)*nu(6)+w(9)*nu(7)+w(10)*nu(4)+w(11)*(nu(3)+nu(4));
dx=zeros(13,1); dx(1:4)=nu(1:4); dx(5:11)=acceleration;
dx(12)=motorPower; dx(13)=dissipation;
casterYawMoment=rC(1)*FC(2)-rC(2)*FC(1)-w(11);
o=[energy;motorPower;dissipation;casterYawMoment;acceleration(3)];
end

