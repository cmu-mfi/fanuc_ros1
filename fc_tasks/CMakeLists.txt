cmake_minimum_required(VERSION 3.0.2)
project(fc_tasks)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

find_package(Boost REQUIRED COMPONENTS system)
find_package(Eigen3 3.3.7 REQUIRED)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  fanuc_lrmate200id7l_moveit_config
  moveit_ros_planning_interface
  moveit_ros_move_group
  tf
  roscpp
  geometry_msgs
  std_msgs
  message_generation
  moveit_msgs
  actionlib
  actionlib_msgs
  fc_msgs
)

###################################
## catkin specific configuration ##
###################################
catkin_package(
   CATKIN_DEPENDS 
     fanuc_lrmate200id7l_moveit_config 
     roscpp
     tf
     message_runtime
     geometry_msgs
     moveit_msgs
     std_msgs
     actionlib_msgs
     rospy
)

###########
## Build ##
###########

## Specify additional locations of header files
include_directories(
  ${catkin_INCLUDE_DIRS}
  include
)

## Declare a C++ executable
add_executable(fc_tasks src/main.cpp src/fc_interface.cpp src/utils.cpp)
# add_executable(service_sample_node src/service_sample_node.cpp)

## Add cmake target dependencies of the executable
add_dependencies(fc_tasks ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
# add_dependencies(service_sample_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(fc_tasks
  ${catkin_LIBRARIES}
  Eigen3::Eigen
  jsoncpp
)

install(DIRECTORY src/fc_tasks/
  DESTINATION ${CATKIN_PACKAGE_PYTHON_DESTINATION}
  PATTERN "*.py"
)

install(PROGRAMS
  src/fc_tasks/IOStates.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
