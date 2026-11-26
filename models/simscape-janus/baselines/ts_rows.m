function [t,v,meta]=ts_rows(ts)
% Preserve MATLAB column-major ordering within each sample, never across time.
t=ts.Time(:);n=numel(t);d=ts.Data;
meta=struct('original_size',size(d),'is_time_first',ts.IsTimeFirst,'components',numel(d)/n);
if ts.IsTimeFirst
 assert(size(d,1)==n,'First dimension must be time');v=reshape(d,n,[]);
else
 assert(size(d,ndims(d))==n,'Last dimension must be time');v=reshape(d,[],n).';
end
assert(size(v,1)==n);assert(all(isfinite(t))&&all(diff(t)>0),'Invalid time grid');
assert(all(isfinite(v(:))),'Nonfinite signal');
assert(isequal(v(1,:),reshape(ts.getdatasamples(1),1,[])),'First sample reshape mismatch');
assert(isequal(v(end,:),reshape(ts.getdatasamples(n),1,[])),'Last sample reshape mismatch');
end
