function [T,meta]=export_native(out,folder,h)
names=out.who;T=table;meta=struct;
for j=1:numel(names)
 if startsWith(names{j},'export_')
  ts=out.get(names{j});[t,v,sm]=ts_rows(ts);
  if isempty(T),T.time_s=t;else,assert(isequal(T.time_s,t),'Export grids differ: %s',names{j});end
  meta.(names{j})=sm;
  for q=1:size(v,2),T.(sprintf('%s_%d',erase(names{j},'export_'),q))=v(:,q);end
 end
end
assert(~isempty(T));assert(max(abs(diff(T.time_s)-h))<1e-9,'Unexpected full-resolution sample grid');
writetable(T,fullfile(folder,'signals_raw.csv'));write_json(fullfile(folder,'signal_shapes.json'),meta);
end
