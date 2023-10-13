# Set compiler flags
set(CMAKE_CXX_STANDARD 14)
add_compile_options(-Wall -Wextra -Wpedantic)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
if (NOT CMAKE_BUILD_TYPE STREQUAL "Debug")
  add_definitions(-O3)
endif(NOT CMAKE_BUILD_TYPE STREQUAL "Debug")

# Find catkin macros and libraries
find_package(catkin REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED COMPONENTS python)

find_package(OpenMP REQUIRED)
if (OpenMP_FOUND)
  add_compile_options("${OpenMP_CXX_FLAGS}")
  add_definitions(-DHAVE_OPENMP=${OpenMP_FOUND})
endif()

# Catkin package macro
catkin_package(
  INCLUDE_DIRS
    nabo
    ${CMAKE_SOURCE_DIR}
    ${EIGEN3_INCLUDE_DIR}
  LIBRARIES
    nabo
  DEPENDS
    Boost
)

########################
## Library definition ##
########################
# Nabo
add_library(nabo
  ${NABO_SRC}
  ${NABO_HEADERS}
)

target_include_directories(nabo
  PUBLIC
    ${CMAKE_SOURCE_DIR}
    ${CMAKE_SOURCE_DIR}/nabo
  SYSTEM
    ${EIGEN3_INCLUDE_DIR}
    ${Boost_INCLUDE_DIRS}
    ${OpenMP_CXX_INCLUDE_DIRS}
)

target_link_libraries(nabo
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
  ${OpenMP_CXX_LIBRARIES}
)

#############
## Install ##
#############
install(
  TARGETS
    nabo
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
  DIRECTORY
    ${CMAKE_SOURCE_DIR}/nabo/
  DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION}/nabo
)

##########
## Test ##
##########
if(CATKIN_ENABLE_TESTING)
  catkin_add_gtest(test_nabo
      tests/empty_test.cpp
  )
  target_include_directories(test_nabo
    PRIVATE
      ${CMAKE_SOURCE_DIR}/nabo
      ${CMAKE_SOURCE_DIR}/tests
    SYSTEM
      ${EIGEN3_INCLUDE_DIR}
      ${catkin_INCLUDE_DIRS}
  )
  target_link_libraries(test_nabo
    gtest_main
    nabo
  )

  ##################
  # Code_coverage ##
  ##################
  find_package(cmake_code_coverage QUIET)
  if(cmake_code_coverage_FOUND)
    add_gtest_coverage(
      TEST_BUILD_TARGETS
        test_nabo
    )
  endif()
endif()

#################
## Clang_tools ##
#################
find_package(cmake_clang_tools QUIET)
if(cmake_clang_tools_FOUND)
  add_default_clang_tooling(
    DISABLE_CLANG_FORMAT
  )
endif(cmake_clang_tools_FOUND)