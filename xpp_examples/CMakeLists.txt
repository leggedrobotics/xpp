cmake_minimum_required(VERSION 2.8.3)
project(xpp_examples)

set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rosbag
  xpp_states
  xpp_msgs
  xpp_ros_conversions
  xpp_vis
  xpp_vis_hyq
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
#   INCLUDE_DIRS 
#   LIBRARIES ${PROJECT_NAME}
#   CATKIN_DEPENDS
#   DEPENDS system_lib
)


###########
## Build ##
###########

## Specify additional locations of header files
include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(monoped_publisher src/monoped_pub.cc)
target_link_libraries(monoped_publisher
  ${catkin_LIBRARIES}
)

add_executable(monoped_bag_builder src/monoped_bag_builder.cc)
target_link_libraries(monoped_bag_builder
  ${catkin_LIBRARIES}
)