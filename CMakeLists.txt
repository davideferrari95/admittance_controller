cmake_minimum_required(VERSION 3.0.2)
project(admittance_controller)

add_compile_options(-std=c++14)

find_package(Eigen3 REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  actionlib
  message_generation

	std_msgs
  geometry_msgs
  trajectory_msgs
  sensor_msgs
  control_msgs
  controller_manager_msgs
  
  moveit_core
  moveit_ros_planning_interface
)

################################################
## Declare ROS messages, services and actions ##
################################################

add_message_files(
  FILES
  joint_trajectory.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
  sensor_msgs
  ${PROJECT_NAME}
)

###################################
## catkin specific configuration ##
###################################

catkin_package(
  INCLUDE_DIRS 
  include
  CATKIN_DEPENDS
	message_runtime
)

###########
## Build ##
###########

include_directories(
  include
  ${EIGEN3_INCLUDE_DIR}
  ${catkin_INCLUDE_DIRS}
)

add_executable(admittance_controller_Node 
	src/admittance_controller/admittance_controller_Node.cpp 
	src/admittance_controller/admittance_controller.cpp)
target_link_libraries(admittance_controller_Node ${catkin_LIBRARIES})
add_dependencies(admittance_controller_Node ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})
