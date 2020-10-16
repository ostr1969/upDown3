# Generic CMakeLists.txt for making an OpenSim-using executable.
# This shows how to use the provided OpenSimConfig.cmake to locate an OpenSim
# installation on your machine so you can use it from your own code.
# You will most likely want to copy some of these lines into your own
# CMakeLists.txt rather than use this one verbatim.

cmake_minimum_required(VERSION 3.2)
project(gcolo)
set(OpenSimMoco_DIR /home/barako/MOCOBIN/cmake)
set(OpenSim_DIR /home/barako/MOCOBIN/opensim-core/cmake)


SET(CMAKE_CXX_COMPILER /opt/rh/devtoolset-7/root/usr/bin/g++) 
SET(CMAKE_C_COMPILER /opt/rh/devtoolset-7/root/usr/bin/gcc) 

# List your source and header files here.
#set(my_source_files JumpingOptimization.cpp PrescribedController.cpp)
set(my_source_files  RegisterTypes_osimMocoJumpGoal.cpp  MocoJumpGoal.cpp mygColo.cpp)
set(fwd_source_files myForward.cpp)
set(4links_source_files 4links.cpp)
set(cmc_source_files cmc.cpp)
#set(ga_source_files JumpingOptimization.cpp)
#set(gafwd_source_files gaForward.cpp)


#set(THREADS_PREFER_PTHREAD_FLAG ON)
#find_package(Threads REQUIRED)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# This depends on OpenSimConfig.cmake being located somewhere predictable
# on your machine. If you have installed it somewhere that CMake won't be
# able to guess, you'll need to tell find_package where to look (with the PATHS
# or HINTS parameters), or specify the installation directory in the
# CMAKE_PREFIX_PATH variable.
find_package(OpenSim REQUIRED)
include_directories(/home/barako/MOCOBIN/sdk/include /home/barako/MOCOBIN/opensim-core/sdk/include)

add_executable(gcolo ${my_source_files} ${my_header_files})
add_executable(fwd ${fwd_source_files} ${my_header_files})
add_executable(build4 ${4links_source_files} ${my_header_files})
add_executable(cmc ${cmc_source_files} ${my_header_files})
#add_executable(pctry ${pctry_source_files} ${my_header_files})
#add_executable(gajump ${ga_source_files} ga.h readinput.h openGA.hpp)
#add_executable(gafwd ${gafwd_source_files}  )

#add_executable(sliding ${sliding_source_files} ${my_header_files})
#target_link_libraries(moco1 ${OpenSim_LIBRARIES} /usr/lib64/libstdc++.so.6  Threads::Threads)
target_link_libraries(gcolo ${OpenSim_LIBRARIES} /usr/lib64/libstdc++.so.6 /home/barako/MOCOBIN/sdk/lib/libosimMoco.so /home/barako/MOCOBIN/ipopt/lib/libipopt.so ~/MOCOBIN/adol-c/lib64/libadolc.so)
target_link_libraries(fwd ${OpenSim_LIBRARIES} /usr/lib64/libstdc++.so.6 /home/barako/MOCOBIN/sdk/lib/libosimMoco.so /home/barako/MOCOBIN/ipopt/lib/libipopt.so  ~/MOCOBIN/adol-c/lib64/libadolc.so)
target_link_libraries(build4 ${OpenSim_LIBRARIES} /usr/lib64/libstdc++.so.6 )
target_link_libraries(cmc ${OpenSim_LIBRARIES} /usr/lib64/libstdc++.so.6 /home/barako/MOCOBIN/sdk/lib/libosimMoco.so /home/barako/MOCOBIN/ipopt/lib/libipopt.so  ~/MOCOBIN/adol-c/lib64/libadolc.so)

