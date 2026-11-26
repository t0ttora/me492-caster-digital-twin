function value=m2_file_hash(file)
% Native MD5 provenance checksum; no Java runtime is required.
% Package transfer uses a separate SHA-256 digest generated outside MATLAB.
value=lower(Simulink.getFileChecksum(file));
end
