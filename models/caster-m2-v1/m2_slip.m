function slip=m2_slip(x,p)
%#codegen
nu=x(5:11); [JL,JR,JC,~,~,eD,nD,eC,nC]=m2_geometry(x,p);
vL=JL*nu;vR=JR*nu;vC=JC*nu;
slip=[eD.'*vL-p(10)*nu(5);nD.'*vL; ...
      eD.'*vR-p(10)*nu(6);nD.'*vR; ...
      eC.'*vC-p(12)*nu(7);nC.'*vC];
end
