cmake_minimum_required(VERSION 3.0.2)
project(planning)

include(CheckCXXCompilerFlag)

CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)

if(COMPILER_SUPPORTS_CXX11)
	set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Wl,--no-as-needed")
elseif(COMPILER_SUPPORTS_CXX0X)
	set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
endif()

find_package(catkin REQUIRED COMPONENTS
  roscpp
  catkin_simple
  std_msgs
  message_generation
  roslib
  victim_signal_gen
  geometry_msgs 
  mav_msgs
)


catkin_simple(ALL_DEPS_REQUIRED)

cs_add_executable(planning_node
  src/planning_node.cpp
)

cs_install()
cs_export()



catkin_package(
  CATKIN_DEPENDS roscpp std_msgs
  CATKIN_DEPENDS message_runtime
)


include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)
