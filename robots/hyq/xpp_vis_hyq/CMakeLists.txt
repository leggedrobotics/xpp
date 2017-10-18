cmake_minimum_required(VERSION 2.8.3)
project(xpp_vis_hyq)

set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  xpp_msgs
  xpp_states
  xpp_ros_conversions
  xpp_vis
)

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
#  INCLUDE_DIRS include
#  LIBRARIES xpp_vis_hyq
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

# Declare a C++ library
add_library(${PROJECT_NAME}
  src/hyqleg_inverse_kinematics.cc
  src/inverse_kinematics_hyq1.cc
  src/inverse_kinematics_hyq2.cc
  src/inverse_kinematics_hyq4.cc
)

## URDF visualizers for all HyQ variants
add_executable(urdf_visualizer_hyq1 src/exe/urdf_visualizer_hyq1.cc)
target_link_libraries(urdf_visualizer_hyq1
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

add_executable(urdf_visualizer_hyq2 src/exe/urdf_visualizer_hyq2.cc)
target_link_libraries(urdf_visualizer_hyq2
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

add_executable(urdf_visualizer_hyq4 src/exe/urdf_visualizer_hyq4.cc)
target_link_libraries(urdf_visualizer_hyq4
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

#############
## Install ##
#############

# Mark library for installation
install(
  TARGETS ${PROJECT_NAME} urdf_visualizer_hyq1 urdf_visualizer_hyq2 urdf_visualizer_hyq4
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

# Mark other files for installation
install(
  DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)