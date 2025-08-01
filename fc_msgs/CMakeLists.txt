cmake_minimum_required(VERSION 3.0.2)
project(fc_msgs)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  tf
  roscpp
  geometry_msgs
  std_msgs
  sensor_msgs
  message_generation
  actionlib
  actionlib_msgs
)

## Generate actions in the 'action' folder
add_action_files(
  DIRECTORY action
  FILES 
  ExecuteCartesianTrajectory.action
  GoToJoints.action
  GoToPose.action
)

## Generate services in the 'srv' folder
add_service_files(
  FILES
  GetPose.srv
  GetPoseStamped.srv
  SetPose.srv
  SetJoints.srv
  ExecuteCartesianTrajectory.srv
  ReadIO.srv
  SetIO.srv
)

## Add messages
add_message_files(
  FILES
  IOState.msg
  IOStateArray.msg
)


## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  geometry_msgs
  std_msgs
  sensor_msgs
  actionlib_msgs
)

###################################
## catkin specific configuration ##
###################################
catkin_package(
   CATKIN_DEPENDS
     roscpp
     tf
     message_runtime
     geometry_msgs
     std_msgs
     sensor_msgs
     actionlib_msgs
)
