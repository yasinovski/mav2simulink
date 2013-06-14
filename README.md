Details
=======


This block will take UDP messages from a user-specified port that contain Mavlink messages, decode those messages and give them as an output to the block. Currently only IMU and visual pose estimation are supported and they need to be sent as debug_vectors with specific strings as name. It is non-trivial to modify the Simulink blocks from the C++ at runtime, so it is in any case impossible to provide an automatic interface as the one in QGroundcontrol.


Installation
============
The latest Mavlink headers are supplied as git a submodule. To generate headers from the submodule:

 mkdir build && cd build
 cmake ..
 make

The block uses boost libraries for UDP communication and multi-threading. To compile the source code for the S-Function block, you will need to use the provided boost libraries with your MATLAB distribution. To do that a matlab script is provided, just run:

 autocompile

on the root folder mav2simulink. Another option is to just call the default boost libraries from the system and use the udp_library generated in the "make" command above:

 mex -Iinclude -lboost_thread -lboost_system -Lbuild -ludp_mavlink mav2simulink.cpp

This gave me problems when linking with the mex-compiled wrapper, since the versions linux and MATLAB libstdc++ are incompatible. 

