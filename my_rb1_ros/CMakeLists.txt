cmake_minimum_required(VERSION 3.0.2)
project(my_rb1_ros)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  geometry_msgs
  nav_msgs
  rospy
  tf
)


add_service_files(
  FILES
  Rotate.srv
)

generate_messages(
  DEPENDENCIES
)

catkin_package(
	CATKIN_DEPENDS
	rospy
)

include_directories( srv
  ${catkin_INCLUDE_DIRS}
)