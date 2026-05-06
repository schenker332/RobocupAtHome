cmake_minimum_required(VERSION 3.0.2)
project(open_door)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  moveit_ros_planning_interface  
  moveit_ros_move_group        
  std_msgs
  std_srvs
  geometry_msgs
  sensor_msgs
  tf
  actionlib
  actionlib_msgs
  control_msgs
  trajectory_msgs
  moveit_msgs
  shape_msgs
  pcl_ros
  pcl_conversions
  message_generation
)

add_action_files(
  DIRECTORY action
  FILES OpenDoor.action
)

generate_messages(
  DEPENDENCIES
  actionlib_msgs
  std_msgs
)

find_package(PCL REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS 
    roscpp 
    moveit_ros_planning_interface  
    moveit_ros_move_group 
    std_msgs 
    std_srvs 
    geometry_msgs 
    sensor_msgs 
    tf 
    actionlib 
    actionlib_msgs 
    control_msgs 
    trajectory_msgs
    moveit_msgs
    shape_msgs
    pcl_ros
    pcl_conversions
    message_runtime
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(open_door_node src/open_door.cpp)
target_link_libraries(open_door_node ${catkin_LIBRARIES} ${PCL_LIBRARIES})
add_dependencies(open_door_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})


install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
