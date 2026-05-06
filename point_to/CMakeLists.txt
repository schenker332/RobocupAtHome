cmake_minimum_required(VERSION 3.0.2)
project(point_to)

find_package(catkin REQUIRED COMPONENTS
  actionlib
  actionlib_msgs
  control_msgs
  moveit_msgs
  moveit_ros_planning_interface
  roscpp
  sensor_msgs
  std_msgs
  message_generation
)

add_action_files(
  DIRECTORY action
  FILES PointTo.action
)

generate_messages(
  DEPENDENCIES
  actionlib_msgs
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS actionlib actionlib_msgs control_msgs moveit_msgs moveit_ros_planning_interface roscpp sensor_msgs std_msgs message_runtime
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(point_to_node src/point_to_node.cpp)
add_dependencies(point_to_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(point_to_node ${catkin_LIBRARIES})