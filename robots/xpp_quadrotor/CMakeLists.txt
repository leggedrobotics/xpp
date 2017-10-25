cmake_minimum_required(VERSION 2.8.3)
project(xpp_quadrotor)

set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
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
#  LIBRARIES quadrotor_description
#  CATKIN_DEPENDS roscpp std_msgs
#  DEPENDS system_lib
)


###########
## Build ##
###########

## Specify additional locations of header files
include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(urdf_visualizer_quadrotor src/exe/urdf_visualizer_quadrotor.cc)
target_link_libraries(urdf_visualizer_quadrotor
  ${catkin_LIBRARIES}
)


#############
## Install ##
#############

# Mark library for installation
install(
  TARGETS urdf_visualizer_quadrotor
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Mark other files for installation
install(
  DIRECTORY launch meshes urdf
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)