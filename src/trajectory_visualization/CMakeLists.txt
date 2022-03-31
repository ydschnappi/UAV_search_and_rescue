cmake_minimum_required(VERSION 3.0.2)
project(trajectory_visualization)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  visualization_msgs
  catkin_simple
  message_generation
  victim_signal_gen
  mav_msgs
  geometry_msgs 
  roslib
)


catkin_simple(ALL_DEPS_REQUIRED)

cs_add_executable(traj_visualize_node
  src/traj_visualize_node.cpp
)

cs_install()
cs_export()


catkin_package(
  LIBRARIES trajectory_visualization
  CATKIN_DEPENDS roscpp visualization_msgs
  CATKIN_DEPENDS message_runtime
)
find_package(victim_signal_gen)

include_directories(
  ${catkin_INCLUDE_DIRS}
)



