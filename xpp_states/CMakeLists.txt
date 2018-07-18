cmake_minimum_required(VERSION 2.8.3)
project(xpp_states)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED)
find_package(Eigen3 REQUIRED)


###################################
## catkin specific configuration ##
###################################
catkin_package(
  INCLUDE_DIRS include ${EIGEN3_INCLUDE_DIR}
  LIBRARIES ${PROJECT_NAME}
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