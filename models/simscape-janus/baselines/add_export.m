function s=add_export(b,n,h)
p=get_param(b,'Parent');s=add_block('simulink/Sinks/To Workspace',[p '/EXPORT_' n],'VariableName',['export_' n],'SaveFormat','Timeseries','SampleTime',num2str(h,16),'MaxDataPoints','inf','Decimation','1');ph=get_param(b,'PortHandles');sh=get_param(s,'PortHandles');add_line(p,ph.Outport(1),sh.Inport(1));
end
