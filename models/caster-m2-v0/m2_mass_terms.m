function [M,bias] = m2_mass_terms(x,p)
%#codegen
% Base rigid body + translating caster lump + absolute swivel and wheel rotors.
[~,~,JC,rH,~,~,~,eC,~]=m2_geometry(x,p);
M=diag([p(1) p(1) p(3) 0 p(5) p(5) p(6)]);
M=M+p(2)*(JC.'*JC);
a=zeros(7,1); a(3:4)=1; M=M+p(4)*(a*a.');
centripetal=-rH*x(7)^2+p(13)*eC*(x(7)+x(8))^2;
bias=p(2)*JC.'*centripetal;
end

