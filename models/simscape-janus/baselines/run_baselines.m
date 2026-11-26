function run_baselines
root=fileparts(mfilename('fullpath'));cd(root);diary(fullfile(root,'baselines_diary.log'));
Simulink.fileGenControl('set','CacheFolder',fullfile(root,'cache'),'CodeGenFolder',fullfile(root,'codegen'),'createDir',true);
status_event('Baseline batch started: real R2026b, ode4, full-step exports.');
for a=[30 -30 0]
 for h=[50 25 12.5]*1e-6
  tag=sprintf('a%+03d_h%g',a,h*1e6);tag=strrep(strrep(strrep(tag,'+','p'),'-','m'),'.','p');m=['native_' tag];folder=fullfile(root,tag);
  if exist(fullfile(folder,'report.json'),'file'),r=jsondecode(fileread(fullfile(folder,'report.json')));if r.completed,status_event(['Skip completed unaffected ' tag]);continue;end;end
  if ~exist(folder,'dir'),mkdir(folder);end
  report=struct('angle_deg',a,'step_s',h,'completed',false,'stage','setup','error','');timer=tic;
  try
   file=fullfile(folder,[m '.slx']);build_native(file,a,h,3);report.stage='simulation';
   save_system(m,file);status_event(['Running ' tag]);
   in=Simulink.SimulationInput(m);in=in.setModelParameter('ReturnWorkspaceOutputs','on','SaveTime','on','TimeSaveName','tout','Timeout',600);
   out=sim(in);report.wall_s=toc(timer);report.end_s=out.tout(end);report.error=out.ErrorMessage;
   save(fullfile(folder,'result.mat'),'out','report','-v7.3');report.stage='export';
   T=export_native(out,folder,h);
   report.completed=abs(report.end_s-3)<1e-9&&isempty(report.error);report.samples=height(T);
   report.caster_initial_deg=T.caster_angle_1(1);report.caster_final_deg=T.caster_angle_1(end);
   report.swivel_final_radps=T.swivel_speed_1(end);report.swivel_final_abs_below_1e4=abs(T.swivel_speed_1(end))<=1e-4;
   report.stage='terminal';
  catch err
   report.wall_s=toc(timer);report.error=getReport(err,'extended','hyperlinks','off');disp(report.error);
  end
  write_json(fullfile(folder,'report.json'),report);status_event(sprintf('%s completed=%d, stage=%s, wall %.1fs',tag,report.completed,report.stage,report.wall_s));
  if bdIsLoaded(m),close_system(m,0);end
 end
end
status_event('Nine-case baseline batch reached terminal outcomes.');diary off
end
