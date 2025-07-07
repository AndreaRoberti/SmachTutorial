cmake_minimum_required(VERSION 3.0.2)
project(PickAndPlace)  # Replace with your package name

find_package(catkin REQUIRED COMPONENTS
  actionlib_msgs
  geometry_msgs
  message_generation
  std_msgs
  rospy
)

## Declare the action files
add_action_files(
  DIRECTORY action
  FILES GoToPose.action
)

## Generate messages and actions
generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
  actionlib_msgs
)

## Declare your catkin package
catkin_package(
  CATKIN_DEPENDS
    actionlib_msgs
    geometry_msgs
    message_runtime
    std_msgs
    rospy
)
