function [rates,o]=m2_energy_power(x,tau,p,w,physicalRhs)
%#codegen
% Work and loss are accounting states, separate from 11 physical states.
nu=x(5:11);[JL,JR,JC,~,rC,eD,~,eC,~]=m2_geometry(x,p);
GL=JL;GR=JR;GC=JC;
GL(:,5)=-p(10)*eD;GR(:,6)=-p(10)*eD;GC(:,7)=-p(12)*eC;
VC=JC*nu;
T=0.5*p(1)*(nu(1:2).'*nu(1:2))+0.5*p(2)*(VC.'*VC) ...
 +0.5*p(3)*nu(3)^2+0.5*p(4)*(nu(3)+nu(4))^2 ...
 +0.5*p(5)*(nu(5)^2+nu(6)^2)+0.5*p(6)*nu(7)^2;
Pin=tau.'*nu(5:6);
D=-w(1:2).'*(GL*nu)-w(3:4).'*(GR*nu)-w(5:6).'*(GC*nu) ...
 +w(7)*nu(5)+w(8)*nu(6)+w(9)*nu(7)+w(10)*nu(4)+w(11)*(nu(3)+nu(4));
rates=[Pin;D];moment=rC(1)*w(6)-rC(2)*w(5)-w(11);
o=[T;Pin;D;moment;physicalRhs(7)];
end
