function wire_axle_inputs(m,src,ports)
for i=1:2
 if i==1,sub=[m '/LEFT_DRIVE'];old=[sub '/Constant'];else,sub=[m '/RIGHT_DRIVE'];old=[sub '/Constant1'];end
 ph=get_param(old,'PortHandles');ln=get_param(ph.Outport(1),'Line');dst=get_param(ln,'DstPortHandle');delete_line(ln);delete_block(old);
 inp=add_block('simulink/Sources/In1',[sub '/AXLE_TORQUE_INPUT'],'Port','1');ip=get_param(inp,'PortHandles');
 for j=1:numel(dst),add_line(sub,ip.Outport(1),dst(j));end
 sp=get_param(sub,'PortHandles');op=get_param(src,'PortHandles');add_line(m,op.Outport(ports(i)),sp.Inport(1));
end
end
