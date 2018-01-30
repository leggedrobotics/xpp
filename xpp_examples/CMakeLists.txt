cmake_minimum_required(VERSION 2.8.3)
project(xpp_examples)

add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rosbag
  xpp_vis
  xpp_hyq
  xpp_quadrotor
)


###################################
## catkin specific configuration ##
###################################
catkin_package()


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

add_executable(quadrotor_bag_builder src/quadrotor_bag_builder.cc)
target_link_libraries(quadrotor_bag_builder
  ${catkin_LIBRARIES}
)


#############
## Install ##
#############
# Mark library for installation
install(
  TARGETS monoped_publisher quadrotor_bag_builder
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Mark other files for installation
install(
  DIRECTORY bags launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)