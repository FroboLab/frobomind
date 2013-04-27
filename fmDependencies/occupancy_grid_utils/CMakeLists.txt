cmake_minimum_required(VERSION 2.8.3)
project(occupancy_grid_utils)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED roscpp message_generation std_msgs geometry_msgs nav_msgs sensor_msgs tf laser_geometry)

## System dependencies are found with CMake's conventions
 find_package(Boost REQUIRED)
 find_package(Eigen REQUIRED)
 include_directories(${Eigen_INCLUDE_DIRS})

## Uncomment this if the package has a setup.py. This macro ensures
## modules and scripts declared therein get installed
# catkin_python_setup()
find_package(PythonLibs REQUIRED)
include_directories(${PYTHON_INCLUDE_PATH})                    

add_library(occupancy_grid_utils src/boost_python_exports.cpp)
add_dependencies(occupancy_grid_utils LocalizedCloud_gencpp NavigationFunction_gencpp OverlayClouds_gencpp std_msgs_gencpp geometry_msgs_gencpp nav_msgs_gencpp sensor_msgs_gencpp)
target_link_libraries(occupancy_grid_utils grid_utils)
target_link_libraries(occupancy_grid_utils ${PYTHON_LIBRARIES})
#link_boost(occupancy_grid_utils python)
set_target_properties(occupancy_grid_utils
  PROPERTIES
  PREFIX "")

#######################################
## Declare ROS messages and services ##
#######################################

## Generate messages in the 'msg' folder
 add_message_files(
   FILES
   LocalizedCloud.msg
   NavigationFunction.msg
   OverlayClouds.msg
 )

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate added messages and services with any dependencies listed here
 generate_messages(
   DEPENDENCIES
   std_msgs
   geometry_msgs
   nav_msgs
   sensor_msgs
 )

###################################################
## Declare things to be passed to other projects ##
###################################################

## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES grid_utils
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
include_directories(include ${catkin_INCLUDE_DIRS})

## Declare a cpp library
add_library(grid_utils src/ray_tracer.cpp
                                src/grid_overlay.cpp
                                src/shortest_path.cpp
                                src/coordinate_conversions.cpp
                                src/combine_grids.cpp
                                src/geometry.cpp
                                src/file.cpp)
add_dependencies(grid_utils occupancy_grid_utils_gencpp std_msgs_gencpp geometry_msgs_gencpp nav_msgs_gencpp sensor_msgs_gencpp)
 target_link_libraries(grid_utils SDL SDL_image ${catkin_LIBRARIES})
 
## Declare a cpp executable
 add_executable(grid_construction_node src/examples/grid_construction_node.cpp)
 add_dependencies(grid_construction_node ${PROJECT_NAME} std_msgs_gencpp geometry_msgs_gencpp nav_msgs_gencpp sensor_msgs_gencpp)
 target_link_libraries(grid_construction_node occupancy_grid_utils)
                      

#############
## Install ##
#############

## Mark executable scripts (Python etc.) for installation
## not required for python when using catkin_python_setup()
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
 install(TARGETS grid_utils
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
 )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#  PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
 catkin_add_gtest(${PROJECT_NAME}-test test/test_grid_utils.cpp)
 if(TARGET ${PROJECT_NAME}-test)
   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
 endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
