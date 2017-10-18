cmake_minimum_required(VERSION 2.8.3)
project(hyqb_description)

## Find catkin macros and libraries
find_package(catkin REQUIRED)

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
#  LIBRARIES hyqb_description
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)

# Mark other files for installation
install(
  DIRECTORY urdf meshes
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)