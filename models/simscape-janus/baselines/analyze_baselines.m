function analyze_baselines
root=fileparts(mfilename('fullpath'));cd(root);folder=fullfile(root,'baseline_analysis');if ~exist(folder,'dir'),mkdir(folder);end
archive=readtable(fullfile(root,'..','evidence_export_20261116_013532','latest_alignment_replay','full_replay.csv'));
results={};comparisons={};tables=cell(3,3);angles=[30 -30 0];steps=[50 25 12.5];
for ai=1:3
 for hi=1:3
  tag=strrep(strrep(strrep(sprintf('a%+03d_h%g',angles(ai),steps(hi)),'+','p'),'-','m'),'.','p');
  r=jsondecode(fileread(fullfile(root,tag,'report.json')));results{end+1}=r;
  if ~r.completed,continue;end
  T=readtable(fullfile(root,tag,'signals_raw.csv'));tables{ai,hi}=T;
  if ai==1
   idx=round(archive.time_s/(steps(hi)*1e-6))+1;assert(max(abs(T.time_s(idx)-archive.time_s))<1e-9);
   c=struct('kind','archive_plus30','candidate',tag,'reference','archived_plus30_25us','common_step_s',1e-4,'samples',height(archive),'max_abs_swivel_rate_delta_radps',max(abs(T.swivel_speed_1(idx)-archive.swivel_speed_c1)),'rms_swivel_rate_delta_radps',sqrt(mean((T.swivel_speed_1(idx)-archive.swivel_speed_c1).^2)),'max_abs_caster_angle_delta_deg',max(abs(T.caster_angle_1(idx)-archive.caster_angle_c1)),'max_abs_x_delta_m',max(abs(T.robot_x_1(idx)-archive.robot_x_c1)),'max_abs_y_delta_m',max(abs(T.robot_y_1(idx)-archive.robot_y_c1)),'max_abs_yaw_delta_deg',max(abs(T.robot_angles_1(idx)-archive.robot_angles_c1)),'swivel_rate_gate_radps',1e-4,'pass',false);c.pass=c.max_abs_swivel_rate_delta_radps<=1e-4;comparisons{end+1}=c;
   C=table(archive.time_s,archive.swivel_speed_c1,T.swivel_speed_1(idx),T.swivel_speed_1(idx)-archive.swivel_speed_c1,'VariableNames',{'time_s','archive_swivel_radps','candidate_swivel_radps','delta_radps'});writetable(C,fullfile(folder,[tag '_archive_common100us.csv']));
  end
 end
end
for ai=1:3
 for hi=1:2
  T=tables{ai,hi};R=tables{ai,3};if isempty(T)||isempty(R),continue;end
  tq=(0:50e-6:3)';it=round(tq/(steps(hi)*1e-6))+1;ir=round(tq/12.5e-6)+1;
  c=struct('kind','step_convergence','candidate',sprintf('angle%d_h%g',angles(ai),steps(hi)),'reference',sprintf('angle%d_h12p5',angles(ai)),'common_step_s',50e-6,'samples',numel(tq),'max_abs_swivel_rate_delta_radps',max(abs(T.swivel_speed_1(it)-R.swivel_speed_1(ir))),'rms_swivel_rate_delta_radps',sqrt(mean((T.swivel_speed_1(it)-R.swivel_speed_1(ir)).^2)),'max_abs_caster_angle_delta_deg',max(abs(T.caster_angle_1(it)-R.caster_angle_1(ir))),'max_abs_x_delta_m',max(abs(T.robot_x_1(it)-R.robot_x_1(ir))),'max_abs_y_delta_m',max(abs(T.robot_y_1(it)-R.robot_y_1(ir))),'max_abs_yaw_delta_deg',max(abs(T.robot_angles_1(it)-R.robot_angles_1(ir))),'swivel_rate_gate_radps',1e-4,'pass',false);c.pass=c.max_abs_swivel_rate_delta_radps<=1e-4;comparisons{end+1}=c;
  writetable(table(tq,T.swivel_speed_1(it),R.swivel_speed_1(ir),T.swivel_speed_1(it)-R.swivel_speed_1(ir),'VariableNames',{'time_s','candidate_swivel_radps','reference_swivel_radps','delta_radps'}),fullfile(folder,[c.candidate '_common50us.csv']));
 end
end
comparisons=[comparisons{:}];write_json(fullfile(folder,'comparisons.json'),comparisons);writetable(struct2table(comparisons),fullfile(folder,'comparisons.csv'));write_json(fullfile(folder,'baseline_outcomes.json'),results);
f=figure('Visible','off','Color','w','Position',[0 0 1400 900]);tiledlayout(3,2);
for ai=1:3
 nexttile;hold on;for hi=1:3,T=tables{ai,hi};if ~isempty(T),plot(T.time_s,T.caster_angle_1,'DisplayName',sprintf('%g us',steps(hi)));end;end;grid on;title(sprintf('Initial caster %+g deg',angles(ai)));ylabel('Caster (deg)');legend;
 nexttile;hold on;for hi=1:3,T=tables{ai,hi};if ~isempty(T),plot(T.time_s,T.swivel_speed_1,'DisplayName',sprintf('%g us',steps(hi)));end;end;grid on;ylabel('Swivel (rad/s)');xlabel('Time (s)');
end
exportgraphics(f,fullfile(folder,'baselines.png'));close(f);status_event(sprintf('Baseline analysis: %d/%d comparisons meet max absolute swivel-rate delta <= 0.0001 rad/s. See baseline_analysis/comparisons.csv.',sum([comparisons.pass]),numel(comparisons)));
end
