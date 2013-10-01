cmake_minimum_required(VERSION 2.8)
project(dtu_smr)

find_package(catkin REQUIRED COMPONENTS roscpp tf)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES dtu_smr
#  CATKIN_DEPENDS msgs roscpp rospy
#  DEPENDS system_lib
)

# include_directories(include)
include_directories(${catkin_INCLUDE_DIRS})
#include_directories(/opt/ros/groovy/stacks/geometry_experimental/tf2/include)

add_executable(smr_control
  src/smr_control_node.cpp
)
target_link_libraries(smr_control ${catkin_LIBRARIES})

add_executable(smr_laser
  src/smr_laser_node.cpp
  src/pugixml.cpp
)
target_link_libraries(smr_laser ${catkin_LIBRARIES})
