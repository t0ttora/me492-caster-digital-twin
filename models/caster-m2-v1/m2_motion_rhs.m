function dx=m2_motion_rhs(x,M,bias,Q)
%#codegen
% Four pose rates and seven coupled acceleration rates.
dx=zeros(11,1);dx(1:4)=x(5:8);dx(5:11)=M\(Q-bias);
end
