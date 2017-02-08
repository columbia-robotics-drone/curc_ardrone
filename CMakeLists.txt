cmake_minimum_required(VERSION 2.8.3)
project(curc_ardrone)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  ardrone_autonomy
  cv_bridge
  dynamic_reconfigure
  geometry_msgs
  sensor_msgs
  std_msgs
  std_srvs
  message_generation
  roscpp
  rospy
)

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS system)

# Compile third party libs
include(ExternalProject)
ExternalProject_Add(thirdparty
	URL ${PROJECT_SOURCE_DIR}/thirdparty/thirdparty.tar.gz
	PREFIX ${CMAKE_BINARY_DIR}/thirdparty
	CONFIGURE_COMMAND ""
	BUILD_COMMAND make
	INSTALL_COMMAND ""
	BUILD_IN_SOURCE 1
)

################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend tag for "message_generation"
##   * add a build_depend and a run_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependency has been pulled in
##     but can be declared for certainty nonetheless:
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
add_message_files(
	FILES
	filter_state.msg
	bounding_box.msg
)

## Generate services in the 'srv' folder
add_service_files(
	DIRECTORY srv
	FILES
	SetReference.srv
	SetMaxControl.srv
	SetInitialReachDistance.srv
	SetStayWithinDistance.srv
	SetStayTime.srv
)

## Generate added messages 
generate_messages(DEPENDENCIES std_msgs)

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## To declare and build dynamic reconfigure parameters within this
## package, follow these steps:
## * In the file package.xml:
##   * add a build_depend and a run_depend tag for "dynamic_reconfigure"
## * In this file (CMakeLists.txt):
##   * add "dynamic_reconfigure" to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * uncomment the "generate_dynamic_reconfigure_options" section below
##     and list every .cfg file to be processed

## Generate dynamic reconfigure parameters in the 'cfg' folder
generate_dynamic_reconfigure_options(
  cfg/AutopilotParams.cfg
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
#  LIBRARIES curc_ardrone
	CATKIN_DEPENDS message_runtime std_msgs ardrone_autonomy
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(${catkin_INCLUDE_DIRS})

# ---------------- state_estimation ----------------
set(STATE_ESTIMATION_SOURCE_FILES
	src/state_estimation/main_state_estimation.cpp
)

set(STATE_ESTIMATION_HEADER_FILES
)

add_executable(curc_state_estimation ${STATE_ESTIMATION_SOURCE_FILES} ${STATE_ESTIMATION_HEADER_FILES})
set_target_properties(curc_state_estimation PROPERTIES COMPILE_FLAGS "-D_LINUX -D_REENTRANT -Wall") 
target_link_libraries(curc_state_estimation ${catkin_LIBRARIES})

# ---------------- tracker ----------------
set(TRACKER_SOURCE_FILES
	src/tracker/main_tracker.cpp
)

set(TRACKER_HEADER_FILES
)

add_executable(curc_tracker ${TRACKER_SOURCE_FILES} ${TRACKER_HEADER_FILES})
target_link_libraries(curc_tracker ${catkin_LIBRARIES})

# ---------------- path_planner ----------------
set(PATH_PLANNER_SOURCE_FILES
	src/path_planner/main_path_planner.cpp
	src/path_planner/PathPlannerNode.cpp
)

set(PATH_PLANNER_HEADER_FILES
	src/path_planner/PathPlannerNode.h
)

add_executable(curc_path_planner ${PATH_PLANNER_SOURCE_FILES} ${PATH_PLANNER_HEADER_FILES})
target_link_libraries(curc_path_planner ${catkin_LIBRARIES})


# ---------------- autopilot ----------------
set(AUTOPILOT_SOURCE_FILES         
	src/autopilot/main_autopilot.cpp  
	src/autopilot/ControlNode.cpp
	src/autopilot/DroneController.cpp
	src/autopilot/KI/KILand.cpp
	src/autopilot/KI/KIAutoInit.cpp
	src/autopilot/KI/KIFlyTo.cpp
)
set(AUTOPILOT_HEADER_FILES        
	src/autopilot/ControlNode.h
	src/autopilot/DroneController.h
	src/autopilot/KI/KILand.h
	src/autopilot/KI/KIAutoInit.h
	src/autopilot/KI/KIFlyTo.h
	src/autopilot/KI/KIProcedure.h
)

# set required libs and headers
include_directories(
  ${CMAKE_BINARY_DIR}/thirdparty/src/thirdparty/TooN/include 
  ${CMAKE_BINARY_DIR}/thirdparty/src/thirdparty/libcvd/include 
  ${CMAKE_BINARY_DIR}/thirdparty/src/thirdparty/gvars3/include 
)  

link_directories(
  ${CMAKE_BINARY_DIR}/thirdparty/src/thirdparty/libcvd/lib 
  ${CMAKE_BINARY_DIR}/thirdparty/src/thirdparty/gvars3/lib 
)

add_executable(drone_autopilot ${AUTOPILOT_SOURCE_FILES} ${AUTOPILOT_HEADER_FILES})
target_link_libraries(drone_autopilot ${catkin_LIBRARIES})
add_dependencies(drone_autopilot thirdparty ${PROJECT_NAME}_gencpp ${PROJECT_NAME}_gencfg)

# ---------------- gui ----------------
set(GUI_SOURCE_FILES
	src/gui/main_gui.cpp
)

set(GUI_HEADER_FILES
)

add_executable(curc_gui ${GUI_SOURCE_FILES} ${GUI_HEADER_FILES})
target_link_libraries(curc_gui ${catkin_LIBRARIES})