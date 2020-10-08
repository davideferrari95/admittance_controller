cmake_minimum_required(VERSION 3.0.2)
project(admittance_controller)

add_compile_options(-std=c++14)

find_package(catkin REQUIRED COMPONENTS
  roscpp
	std_msgs
	trajectory_msgs
  geometry_msgs
	control_msgs
	actionlib
)


################################################
## Declare ROS messages, services and actions ##
################################################

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
#   std_msgs  # Or other packages containing msgs
# )

###################################
## catkin specific configuration ##
###################################

catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES admittance_controller
#  CATKIN_DEPENDS roscpp
#  DEPENDS system_lib
)

###########
## Build ##
###########

include_directories(
	include
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ library
# add_library(${PROJECT_NAME}
#   src/${PROJECT_NAME}/admittance_controller.cpp
# )


add_executable(admittance_controller_Node 
	src/admittance_controller/admittance_controller_Node.cpp 
	src/admittance_controller/admittance_controller.cpp)
target_link_libraries(admittance_controller_Node ${catkin_LIBRARIES})
add_dependencies(admittance_controller_Node ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})
