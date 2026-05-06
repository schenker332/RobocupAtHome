cmake_minimum_required(VERSION 3.0.2)
project(audio_detection)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  messages
)

catkin_package()

catkin_install_python(PROGRAMS
  scripts/audio.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
