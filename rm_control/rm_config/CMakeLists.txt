cmake_minimum_required(VERSION 3.0.2)
project(rm_config)

find_package(catkin REQUIRED
        COMPONENTS
        )
###################################
## catkin specific configuration ##
###################################
catkin_package(
        CATKIN_DEPENDS
)
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)
