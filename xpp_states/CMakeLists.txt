cmake_minimum_required(VERSION 2.8.3)
project(xpp_states)

set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")

## Find catkin macros and libraries
find_package(catkin REQUIRED)

## System dependencies are found with CMake's conventions
find_package(Eigen3 REQUIRED)


###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
   INCLUDE_DIRS include ${EIGEN3_INCLUDE_DIR}
   LIBRARIES ${PROJECT_NAME}
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
include_directories(
  include
  ${EIGEN3_INCLUDE_DIR}
)

add_library(${PROJECT_NAME}
  src/state.cc
  src/joints.cc
  src/robot_state_cartesian.cc
  src/robot_state_joint.cc
)
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

#############
## Install ##
#############

# Mark library for installation
install(
  TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Mark header files for installation
install(
  DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)