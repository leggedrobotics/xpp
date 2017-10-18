cmake_minimum_required(VERSION 2.8.3)
project(xpp_vis)

set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rosbag
  tf
  kdl_parser
  robot_state_publisher
  xpp_ros_conversions
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
   LIBRARIES ${PROJECT_NAME}
   CATKIN_DEPENDS xpp_ros_conversions roscpp
#   DEPENDS system_lib
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
  src/urdf_visualizer.cc
  src/cartesian_joint_converter.cc
  src/rviz_robot_builder.cc
  src/rviz_terrain_builder.cc
)
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
)


# some executable nodes
add_executable(rviz_marker_node src/exe/rviz_marker_node.cc)
target_link_libraries(rviz_marker_node
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

add_executable(monped_publisher demo/monoped_pub.cc)
target_link_libraries(monped_publisher
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

add_executable(monped_bag_builder demo/monoped_bag_builder.cc)
target_link_libraries(monped_bag_builder
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
)


#############
## Testing ##
#############

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread")
catkin_add_gtest(${PROJECT_NAME}_test
  test/test_xpp_vis.cc 
  test/utest.cc
)