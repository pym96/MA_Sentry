cmake_minimum_required(VERSION 3.0.2)
project(rm_decision)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  behaviortree_cpp_v3
  actionlib_msgs
  sensor_msgs
  nav_msgs
  roscpp
  tf
)


set(CMAKE_CXX_STANDARD 17)
###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
 INCLUDE_DIRS include
 CATKIN_DEPENDS geometry_msgs roscpp sensor_msgs nav_msgs tf behaviortree_cpp_v3 actionlib_msgs
)

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
include
  ${catkin_INCLUDE_DIRS}
  ${BEHAVIORTREEV3_INCLUDE_DIRS}
)

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
add_executable(${PROJECT_NAME}_test_node src/random_radius_explore.cc src/patrol.cc src/random_explore.cc)
add_executable(${PROJECT_NAME}_final_node src/final_decision.cc)
add_executable(${PROJECT_NAME}_finalmb_node src/final_decision_mb.cc)
# add_dependencies(${PROJECT_NAME}_test_node ${catkin_LIBRARIES})
## Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME}_test_node
  ${catkin_LIBRARIES}  
)

target_link_libraries(${PROJECT_NAME}_final_node 
  ${catkin_LIBRARIES}  
)

target_link_libraries(${PROJECT_NAME}_finalmb_node 
  ${catkin_LIBRARIES}  
)