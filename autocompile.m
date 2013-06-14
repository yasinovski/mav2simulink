% This script will determine the boost libraries in the local system
% to be used and compile the mex file.

path2libs = [matlabroot, '/bin/', computer('arch')];

libs = dir(path2libs);
libs = {libs(:).name};

r1 = regexp(libs, '[A-z]boost_thread');
r2 = regexp(libs, '[A-z]boost_system');

boost_threads = [path2libs,'/',libs{~cellfun(@isempty,r1)},  ' '];
boost_system  = [path2libs, '/', libs{~cellfun(@isempty,r2)},  ' '];


sources = ['mav2simulink.cpp ',  'udp_mavlink_rec.cpp'];
                    
compile_str = ['mex -g -Iinclude ', boost_system, boost_threads, sources];

clc
fprintf('\n============ COMPILATON STARTING ============\n');
eval( compile_str )
fprintf('============ COMPILATON ENDED    ============\n');
