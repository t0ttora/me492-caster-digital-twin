function b=find_named(blocks,n)
b='';for i=1:numel(blocks),if strcmp(regexprep(strtrim(get_param(blocks{i},'Name')),'\s+',' '),n),b=blocks{i};break;end;end;assert(~isempty(b),'Missing block %s',n);
end
