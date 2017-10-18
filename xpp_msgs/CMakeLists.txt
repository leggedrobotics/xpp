cmake_minimum_required(VERSION 2.8.3)
project(xpp_msgs)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  message_generation
  std_msgs
  geometry_msgs
  sensor_msgs
)

## Generate messages in the 'msg' folder
add_message_files(
  FILES
  StateLin3d.msg
  State6d.msg
  UserCommand.msg
  OptParameters.msg
  RobotStateCartesianTrajectory.msg
  RobotStateCartesian.msg
  RobotStateJoint.msg
  TerrainInfo.msg
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
  sensor_msgs
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
   INCLUDE_DIRS include
#   LIBRARIES
   CATKIN_DEPENDS
     message_runtime
     std_msgs
     geometry_msgs
     sensor_msgs 
#  DEPENDS system_lib
)

#############
## Install ##
#############

# Mark topic names header files for installation
install(
  DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)
