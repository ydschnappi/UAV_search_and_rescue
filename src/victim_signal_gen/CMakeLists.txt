cmake_minimum_required(VERSION 3.0.2)
project(victim_signal_gen)

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
)

add_message_files(
  FILES
  SignalArray.msg
  VictimSignal.msg
  VictimCoord.msg
  VictimArray.msg
)


catkin_simple(ALL_DEPS_REQUIRED)

cs_add_executable(victim_signal_node
  src/victim_signal_node.cpp
  src/victim_signal_gen.cpp
)


cs_install()
cs_export()





catkin_package(
  CATKIN_DEPENDS roscpp std_msgs message_runtime
)

#find_package(Eigen3)


include_directories(

  ${catkin_INCLUDE_DIRS}
)

