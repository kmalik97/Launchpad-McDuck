cmake_minimum_required(VERSION 3.0.2)
project(launchpad)

find_package(catkin
  REQUIRED COMPONENTS
  rospy
  std_msgs
  sensor_msgs
  message_generation
)

add_service_files(
  FILES
  motionLogic.srv
  snapshot.srv
  measurement.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
  sensor_msgs
)

catkin_install_python(
  PROGRAMS
  scripts/motion_logic.py
  scripts/motion_interface.py
  scripts/camera_interface.py
  scripts/image_processing.py
  scripts/camera_publisher.py
  scripts/camera_subscriber.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

catkin_package(
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)


