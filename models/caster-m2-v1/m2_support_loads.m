function normal=m2_support_loads(x,p)
%#codegen
% Quasi-static force and moment balance about drive-axle ground origin A.
cx=p(14)-p(13)*cos(x(4)); cy=p(15)-p(13)*sin(x(4));
Nc=p(16)*(p(1)*p(7)+p(2)*cx)/cx;
difference=2*p(16)*(p(1)*p(8)+p(2)*cy)/p(11)-2*Nc*cy/p(11);
Nl=0.5*((p(1)+p(2))*p(16)-Nc+difference);
Nr=0.5*((p(1)+p(2))*p(16)-Nc-difference);
assert(Nl>0 && Nr>0 && Nc>0);normal=[Nl;Nr;Nc];
end
