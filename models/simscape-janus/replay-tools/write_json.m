function write_json(p,v)
f=fopen(p,'w');assert(f>=0,'Cannot open %s',p);c=onCleanup(@()fclose(f));fprintf(f,'%s',jsonencode(v,PrettyPrint=true));
end
