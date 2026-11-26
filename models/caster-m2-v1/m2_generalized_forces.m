function Q=m2_generalized_forces(x,tau,p,w)
%#codegen
% The SAME ground forces act on body, swivel and rolls via slip Jacobians.
[JL,JR,JC,~,~,eD,~,eC,~]=m2_geometry(x,p);
GL=JL;GR=JR;GC=JC;
GL(:,5)=-p(10)*eD;GR(:,6)=-p(10)*eD;GC(:,7)=-p(12)*eC;
Q=GL.'*w(1:2)+GR.'*w(3:4)+GC.'*w(5:6);
Q(5)=Q(5)+tau(1)-w(7);Q(6)=Q(6)+tau(2)-w(8);Q(7)=Q(7)-w(9);
Q(4)=Q(4)-w(10); % internal bearing: relative swivel only
Q(3:4)=Q(3:4)-w(11); % ground bore: absolute psi=theta+phi
end
