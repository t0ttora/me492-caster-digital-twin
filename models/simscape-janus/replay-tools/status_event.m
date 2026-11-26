function status_event(msg)
root=fileparts(mfilename('fullpath'));stamp=char(datetime('now','TimeZone','Europe/Istanbul','Format','yyyy-MM-dd HH:mm:ss Z'));
f=fopen(fullfile(root,'STATUS.md'),'a');fprintf(f,'\n- %s: %s\n',stamp,msg);fclose(f);fprintf('%s %s\n',stamp,msg);
end
