cmake_minimum_required(VERSION 3.0.2)
project(messages)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  message_generation
  actionlib_msgs
  roscpp
  rospy
  std_msgs
  vision_msgs  # WICHTIG: Hinzugefügt für BoundingBox2D
)

################################################
## Declare ROS messages, services and actions ##
################################################

## Generate messages in the 'msg' folder
add_message_files(
  FILES
  GuestInfo.msg
  GuestList.msg
  PersonPose.msg 
  AudioData.msg
  Seat.msg       
  SeatArray.msg
)

## Generate actions in the 'action' folder
add_action_files(
  FILES
  OpenDoor.action
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
  vision_msgs  # WICHTIG: Damit ROS den Typ in PersonPose kennt
  actionlib_msgs
)

###################################
## catkin specific configuration ##
###################################
catkin_package(
# INCLUDE_DIRS include
# LIBRARIES messages
  CATKIN_DEPENDS
    geometry_msgs
    message_runtime
    actionlib_msgs
    roscpp
    rospy
    std_msgs
    vision_msgs # WICHTIG: Exportiert die Dependency für andere Nodes
)

###########
## Build ##
###########

## Specify additional locations of header files
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)