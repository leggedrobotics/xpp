cmake_minimum_required(VERSION 2.8.3)
project(xpp_vis)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  keyboard
  joy
  tf
  kdl_parser
  robot_state_publisher
  rosbag
  xpp_ros_conversions
)

## System dependencies are found with CMake's conventions
find_package(Eigen3 REQUIRED)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wfatal-errors -std=c++11" )

## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend and a run_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependencies might have been
##     pulled in transitively but can be declared for certainty nonetheless:
##     * add a build_depend tag for "message_generation"
##     * add a run_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
# generate_messages(
#   DEPENDENCIES
#   std_msgs
# )

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
   CATKIN_DEPENDS
   #DEPENDS eigen
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

# Declare a C++ library
add_library(${PROJECT_NAME}
  src/urdf_visualizer.cc
  src/cartesian_joint_converter.cc
  src/rviz_marker_builder.cc
  src/hyqleg_inverse_kinematics.cc
  src/hyq_inverse_kinematics.cc
  src/biped_inverse_kinematics.cc
  src/monoped_inverse_kinematics.cc
  src/nlp_user_input_node.cc
)
target_link_libraries(xpp_vis
  ${catkin_LIBRARIES}
)



add_executable(nlp_user_input_node
    src/exe/nlp_user_input_node_exe.cc
)
target_link_libraries(nlp_user_input_node 
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

add_executable(robot_state_publisher
    src/exe/robot_state_publisher.cc
)
target_link_libraries(robot_state_publisher
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
)



## URDF visualizers
add_executable(urdf_visualizer_monoped src/exe/urdf_visualizer_monoped.cc)
target_link_libraries(urdf_visualizer_monoped
  xpp_vis
  ${catkin_LIBRARIES}
)

add_executable(urdf_visualizer_biped src/exe/urdf_visualizer_biped.cc)
target_link_libraries(urdf_visualizer_biped
  xpp_vis
  ${catkin_LIBRARIES}
)

add_executable(urdf_visualizer_hyq src/exe/urdf_visualizer_hyq.cc)
target_link_libraries(urdf_visualizer_hyq
  xpp_vis
  ${catkin_LIBRARIES}
)


## other visualizers
add_executable(rviz_marker_node src/exe/rviz_marker_node.cc)
target_link_libraries(rviz_marker_node
  xpp_vis
  ${catkin_LIBRARIES}
)

## misc
add_executable(rosbag_traj_combiner src/exe/rosbag_traj_combiner.cc)
target_link_libraries(rosbag_traj_combiner
  ${catkin_LIBRARIES}
)


#add_executable(base_state_visualizer src/exe/base_state_visualizer.cc)
#target_link_libraries(base_state_visualizer
#  xpp_vis
#  ${catkin_LIBRARIES}
#)

#add_executable(initial_state_visualizer src/exe/initial_state_visualizer.cc)
#target_link_libraries(initial_state_visualizer
#  xpp_vis
#  ${catkin_LIBRARIES}
#)


#add_executable(gazebo_state_publisher src/exe/gazebo_state_publisher.cc)
#target_link_libraries(gazebo_state_publisher
#  xpp_vis
#  ${catkin_LIBRARIES}
#)
#
#add_executable(example_state_publisher src/exe/example_state_publisher.cc)
#target_link_libraries(example_state_publisher
#  xpp_vis
#  ${catkin_LIBRARIES}
#)
 
## add msg dependencies



