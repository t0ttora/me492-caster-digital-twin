function c=m2_contact_diagnostics(x,p,slip,normal,w)
%#codegen
[~,~,~,~,~,eD,nD,eC,nC]=m2_geometry(x,p);
c=[slip;normal;[eD nD].'*w(1:2);[eD nD].'*w(3:4);[eC nC].'*w(5:6)];
end
