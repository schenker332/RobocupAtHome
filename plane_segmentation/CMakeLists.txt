cmake_minimum_required(VERSION 3.0.2)
project(plane_segmentation)

# Find all required packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  pcl_ros
  pcl_conversions
  tf
  cv_bridge
  image_transport
  moveit_core
  moveit_ros_planning_interface
)

find_package(PCL REQUIRED)

# Catkin package definition
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS 
    roscpp 
    sensor_msgs 
    pcl_ros 
    pcl_conversions 
    tf 
    cv_bridge 
    image_transport
    moveit_core
    moveit_ros_planning_interface
)

# Include directories
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

# PCL Definitions
add_definitions( ${PCL_DEFINITIONS} )

# Library (plane_segmentation.cpp)
add_library(${PROJECT_NAME}
  src/plane_segmentation.cpp
)

# Executable (the main file)
add_executable(${PROJECT_NAME}_node 
  src/applications/plane_segmentation_node.cpp
)

# Link executable to the library
target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
  ${PROJECT_NAME}
)

# Link library to its dependencies
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)