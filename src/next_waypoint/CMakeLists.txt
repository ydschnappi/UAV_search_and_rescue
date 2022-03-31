cmake_minimum_required(VERSION 3.0.2)
project(next_waypoint)

add_definitions(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  eigen_catkin
  catkin_simple
  mav_msgs
  mav_planning_msgs
  mav_trajectory_generation
  mav_trajectory_generation_ros
  mav_visualization
  roslib
  roscpp
  std_msgs
  message_generation
)


#add_message_files(
#  FILES
#  Nextpoint.msg
#)


catkin_simple(ALL_DEPS_REQUIRED)

cs_add_executable(next_node
        src/next_node.cc
        )

cs_install()
cs_export()


# generate_messages(
#   DEPENDENCIES
#   std_msgs  
# )

catkin_package(
  CATKIN_DEPENDS message_runtime
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)



