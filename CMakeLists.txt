#
# Author: 
#
 
cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
set(ROS_BUILD_TYPE RelWithDebInfo)

rosbuild_init()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)


# ------------------- add dynamic reconfigure api ------------------------------------
#rosbuild_find_ros_package(dynamic_reconfigure)
#include(${dynamic_reconfigure_PACKAGE_PATH}/cmake/cfgbuild.cmake)
#gencfg()




# --------------------------- NodeGenerator --------------------------------
# set header ans source files
set(NODEGENERATOR_SOURCE_FILES      
  src/NodeGenerator.cpp
)
set(NODEGENERATOR_HEADER_FILES    
  src/KeyBoardMacro.h
  src/NodeGenerator.h
)

# build!
rosbuild_add_executable(node_generator ${NODEGENERATOR_SOURCE_FILES} ${NODEGENERATOR_HEADER_FILES})
rosbuild_add_compile_flags(node_generator -D_LINUX -D_REENTRANT -Wall  -O3 -march=nocona -msse3) 

# do QT stuff
#ADD_DEFINITIONS( -Wall )
#find_package(Qt4 REQUIRED)
#include(${QT_USE_FILE})
#include_directories(${CMAKE_CURRENT_BINARY_DIR})

# ---------------------------- Messages & Services --------------------------------------------------
#uncomment if you have defined messages
rosbuild_genmsg()
#uncomment if you have defined services
#rosbuild_gensrv()
