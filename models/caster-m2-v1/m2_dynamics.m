function [dx,o] = m2_dynamics(x,tau,p,w)
%#codegen
[M,bias]=m2_mass_terms(x,p);Q=m2_generalized_forces(x,tau,p,w);
physical=m2_motion_rhs(x,M,bias,Q);[rates,o]=m2_energy_power(x,tau,p,w,physical);
dx=[physical;rates];
end
