cmake_minimum_required(VERSION 2.8.3)
project(xpp_vis)

add_compile_options(-std=c++11)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  tf
  kdl_parser
  robot_state_publisher
  visualization_msgs
  xpp_states
  xpp_msgs
)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
catkin_package(
   INCLUDE_DIRS include
   LIBRARIES ${PROJECT_NAME}
   CATKIN_DEPENDS xpp_states xpp_msgs
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


#############
## Install ##
#############
# Mark library for installation
install(
  TARGETS ${PROJECT_NAME} rviz_marker_node
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
  DIRECTORY launch rviz
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)


#############
## Testing ##
#############
if (CATKIN_ENABLE_TESTING)
  catkin_add_gtest(${PROJECT_NAME}_test
    test/gtest_main.cc 
    test/rviz_robot_builder_test.cc
  )
  target_link_libraries(${PROJECT_NAME}_test
    ${PROJECT_NAME} 
    ${catkin_LIBRARIES}
  )
endif()
