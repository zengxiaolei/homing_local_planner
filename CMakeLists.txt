cmake_minimum_required(VERSION 3.0.2)
project(homing_local_planner)

set(CMAKE_BUILD_TYPE Release)

find_package(catkin REQUIRED COMPONENTS
  base_local_planner
  cmake_modules
  costmap_2d
  dynamic_reconfigure
  geometry_msgs
  message_generation
  nav_core
  nav_msgs
  pluginlib
  roscpp
  rospy
  std_msgs
  tf2
  tf2_ros
  tf2_eigen
  visualization_msgs
)
find_package(Eigen REQUIRED)


################################################
## Declare ROS messages, services and actions ##
################################################
generate_dynamic_reconfigure_options(
  cfg/HomingLocalPlannerReconfigure.cfg
)


###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  CATKIN_DEPENDS
	base_local_planner
	costmap_2d
  dynamic_reconfigure
	geometry_msgs
  message_runtime
	nav_core
	nav_msgs
	pluginlib
	roscpp
	std_msgs
  tf2
  tf2_ros
  visualization_msgs
#  INCLUDE_DIRS include
#  LIBRARIES homing_local_planner
#  CATKIN_DEPENDS base_local_planner cmake_modules costmap_2d dynamic_reconfigure nav_core nav_msgs pluginlib roscpp rospy std_msg tf2_ros
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_library(homing_local_planner 
src/homing_local_planner_ros.cpp
src/visualization.cpp
src/homing_config.cpp
src/obstacles.cpp)

add_dependencies(homing_local_planner ${PROJECT_NAME}_gencfg)

add_dependencies(homing_local_planner ${catkin_EXPORTED_TARGETS})
target_link_libraries(homing_local_planner
${EXTERNAL_LIBS}
${catkin_LIBRARIES}
)

#############
## Install ##
#############
install(DIRECTORY
  cfg
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
