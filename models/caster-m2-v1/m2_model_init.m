function m2_model_init(model)
% Source lookup works when opening the SLX directly from /models.
root=fileparts(fileparts(get_param(model,'FileName')));
assert(isfile(fullfile(root,'m2_contact.m')),'ME492:MissingSources', ...
 'Keep the models folder together with the package MATLAB sources.');
addpath(root);
% Preserve model workspace experiments. run_m2 owns JSON/scenario replay values.
end
