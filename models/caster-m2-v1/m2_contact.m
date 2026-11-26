function [w,c] = m2_contact(x,p,casterEnabled)
%#codegen
% Same M2 contact physics, separated into visible, executable stages.
normal=m2_support_loads(x,p);slip=m2_slip(x,p);
w=m2_friction(x,p,slip,normal,casterEnabled);
c=m2_contact_diagnostics(x,p,slip,normal,w);
end
