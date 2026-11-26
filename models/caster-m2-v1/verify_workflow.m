function evidence=verify_workflow()
% Tests the user-facing run lifecycle in a separate MATLAB process.
root=fileparts(mfilename('fullpath'));sourceModel='Me492CasterM2V1';
originalPath=path;originalFolder=pwd;cleanup=onCleanup(@() restore(originalPath,originalFolder));
checks=struct('name',{},'passed',{},'value',{});
evalin('base','restoredefaultpath');cd(fullfile(root,'models'));load_system(fullfile(root,'models',[sourceModel '.slx']));
assert(isempty(which('m2_contact')),'Clean-path precondition was not met.');
out=sim(sourceModel,'ReturnWorkspaceOutputs','on');
record('clean_models_folder_direct_simulation',numel(out.get('m2_states').Time)>10,numel(out.get('m2_states').Time));
stateSeries=out.get('m2_states');stateData=squeeze(stateSeries.Data);
if size(stateData,1)~=numel(stateSeries.Time),stateData=stateData.';end
record('direct_simulation_has_13_states',size(stateData,2)==13,size(stateData,2));
close_system(sourceModel,0);
% Separate package copy for saved-manual-edit and sample ownership tests.
testsFolder=fullfile(root,'workflow_tests');if ~isfolder(testsFolder),mkdir(testsFolder);end
sandbox=tempname(testsFolder);mkdir(sandbox);mkdir(fullfile(sandbox,'models'));
for pattern={'*.m','*.json'}
 files=dir(fullfile(root,pattern{1}));for k=1:numel(files),copyfile(fullfile(root,files(k).name),sandbox);end,end
sandboxModel=fullfile(sandbox,'models',[sourceModel '.slx']);copyfile(fullfile(root,'models',[sourceModel '.slx']),sandboxModel);
baselineModel=fullfile(sandbox,'models','Me492CasterM2.slx');copyfile(fullfile(root,'models','Me492CasterM2.slx'),baselineModel);
d=jsondecode(fileread(fullfile(sandbox,'dynamics_si.json')));d.result_sample_dt_s=0.02;
write_json(fullfile(sandbox,'dynamics_si.json'),d);
addpath(sandbox,'-begin');cd(sandbox);clear run_m2 m2_load_parameters m2_model_init;
load_system(sandboxModel);tag='Saved manual edit is preserved by replay';
set_param(sourceModel,'Description',tag);save_system(sourceModel);before=m2_file_hash(sandboxModel);
run_m2('aligned_forward');
record('runner_keeps_open_saved_model',bdIsLoaded(sourceModel),double(bdIsLoaded(sourceModel)));
record('runner_preserves_saved_manual_description',strcmp(get_param(sourceModel,'Description'),tag),get_param(sourceModel,'Description'));
record('runner_preserves_source_slx_bytes',strcmp(before,m2_file_hash(sandboxModel)),m2_file_hash(sandboxModel));
c=readtable(fullfile(sandbox,'commands','aligned_forward_torque.csv'));s=readtable(fullfile(sandbox,'results','aligned_forward_states.csv'));
record('commands_use_10ms_grid',height(c)==401 && max(abs(diff(c.time_s)-0.01))<1e-12,height(c));
record('results_use_independent_20ms_grid',height(s)==201 && max(abs(diff(s.time_s)-0.02))<1e-12,height(s));
set_param(sourceModel,'Description','Unsaved test change');caught=false;
try,run_m2('aligned_forward');catch err,caught=contains(err.message,'Save manual model edits');end
record('runner_rejects_unsaved_edits',caught,double(caught));
close_system(sourceModel,0); % only the isolated test copy
leaks=dir(fullfile(sandbox,'models','M2Replay_*.slx'));
record('temporary_replay_copy_cleaned',isempty(leaks),numel(leaks));
% Rebuild beside a same-named, unsaved baseline model in this isolated process.
movefile(sandboxModel,fullfile(sandbox,'models','Me492CasterM2V1_saved.slx'));
load_system(baselineModel);baselineChecksum=m2_file_hash(baselineModel);
baselineTag='Unsaved baseline edit must survive build';set_param('Me492CasterM2','Description',baselineTag);
build_m2;
record('builder_keeps_same_named_open_baseline',bdIsLoaded('Me492CasterM2'),double(bdIsLoaded('Me492CasterM2')));
record('builder_preserves_unsaved_baseline_edit',strcmp(get_param('Me492CasterM2','Dirty'),'on') ...
 && strcmp(get_param('Me492CasterM2','Description'),baselineTag),get_param('Me492CasterM2','Description'));
record('builder_preserves_baseline_slx_bytes',strcmp(baselineChecksum,m2_file_hash(baselineModel)),m2_file_hash(baselineModel));
builtOut=sim(sourceModel,'ReturnWorkspaceOutputs','on');builtSeries=builtOut.get('m2_states');builtData=squeeze(builtSeries.Data);
if size(builtData,1)~=numel(builtSeries.Time),builtData=builtData.';end
record('rebuilt_model_compiles_and_simulates',size(builtData,2)==13 && all(isfinite(builtData),'all'),size(builtData,2));
close_system(sourceModel,0);close_system('Me492CasterM2',0); % isolated test models only
evidence=struct('all_passed',all([checks.passed]),'checks',checks,'test_package',sandbox, ...
 'scope','Workflow and synthetic model execution; not robot validation.');
write_json(fullfile(root,'workflow_verification_evidence.json'),evidence);
fprintf('Workflow verification: %d/%d checks passed.\n',sum([checks.passed]),numel(checks));
assert(evidence.all_passed,'ME492:WorkflowFailed','A workflow check failed.');
    function record(name,ok,value)
        checks(end+1)=struct('name',name,'passed',logical(ok),'value',value);
        fprintf('%s: passed=%d\n',name,ok);
    end
end
function restore(pwdPath,folder),path(pwdPath);cd(folder);end
function write_json(file,data)
fid=fopen(file,'w','n','UTF-8');fprintf(fid,'%s',jsonencode(data,'PrettyPrint',true));fclose(fid);
end
