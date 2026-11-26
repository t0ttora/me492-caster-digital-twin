function h=file_sha256(path)
fid=fopen(path,'rb');assert(fid>=0);c=onCleanup(@()fclose(fid));md=java.security.MessageDigest.getInstance('SHA-256');
while ~feof(fid),b=fread(fid,1048576,'*uint8');md.update(b);end
h=lower(reshape(dec2hex(typecast(md.digest(),'uint8'),2).',1,[]));
end
