function polish_m2(model)
% Rendering changes only: keep executable equations, ports and logging intact.
lines=find_system(model,'FindAll','on','Type','line');
for k=1:numel(lines),set_param(lines(k),'Name','');end
for name={'WheelGroundForces','BodyCasterInertiaDynamics'}
 sub=[model '/' name{1}];set_param(sub,'ContentPreviewEnabled','off');
 mask=Simulink.Mask.get(sub);if isempty(mask),mask=Simulink.Mask.create(sub);end
 if strcmp(name{1},'WheelGroundForces')
  mask.Display=['disp(''Loads -> slip -> passive contact'');' ...
   'port_label(''input'',1,''x13'');port_label(''input'',2,''p32 [SI]'');' ...
   'port_label(''input'',3,''enabled'');port_label(''output'',1,''F / tau'');port_label(''output'',2,''slip / N'');'];
 else
  mask.Display=['disp(''M * nuDot + bias = Q'');' ...
   'port_label(''input'',1,''x13'');port_label(''input'',2,''tauL/R [N m]'');' ...
   'port_label(''input'',3,''p32 [SI]'');port_label(''input'',4,''F / tau'');' ...
   'port_label(''output'',1,''dx13/dt'');port_label(''output'',2,''energy / power'');'];
 end
end
% Place both state-return branches below the main left-to-right flow.
src=get_param([model '/MechanicalState'],'PortHandles');s=get_param(src.Outport(1),'Position');
for name={'WheelGroundForces','BodyCasterInertiaDynamics'}
 ph=get_param([model '/' name{1}],'PortHandles');dest=get_param(ph.Inport(1),'Position');h=get_param(ph.Inport(1),'Line');
 set_param(h,'Points',[s;s+[40 0];s(1)+40 535;dest(1)-35 535;dest(1)-35 dest(2);dest]);
end
end
