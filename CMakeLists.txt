cmake_minimum_required(VERSION 2.8.3)
project(basketball_trajectory_srv)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  basketball_msgs
  geometry_msgs
  nav_msgs
  roscpp
  rospy
  tf
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES basketball_trajectory_srv
  CATKIN_DEPENDS geometry_msgs nav_msgs roscpp rospy tf
  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(robot_trajectory_srv src/robot_trajectory.cpp)
target_link_libraries(robot_trajectory_srv ${catkin_LIBRARIES})
add_dependencies(robot_trajectory_srv basketball_msgs_gencpp)
