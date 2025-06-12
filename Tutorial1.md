# Tutorial 1 - Configuring the Interface for another Fanuc Robot Arm

> Note: This tutorial assumes that the new package is built in a container with the same image as fanuc_ros1. If using a different environment, all dependencies must be satisfied manually as Noetic is now EOL. 

## Make the following changes 
1. Find the required support package and moveit config for the robot arm at [fanuc](https://github.com/ros-industrial/fanuc). 
1. Change the package.xml dependencies of <i>fc_tasks</i> and <i>fc_launch</i> to match the new moveit config package (fanuc_lrmate200id7l_moveit_config).
1. Use the urdf file from the downloaded support package to find DH parameters of the new robot arm. Update the .json file in '/fc_tasks/config' with the new values. 
1. In 'fc_launch/launch/moveit.launch', change the 'fanuc_lrmate200id7l_moveit_config' to the new moveit config package and update the 'robot_ip'.
1. Inside the moveit_config package, add the namespaced controllers to the 'config/controllers.yaml' file. Refer 'fanuc_lrmate200id7l_moveit_config'. 
1. Go through the moveit_config launch and config files to see if any file are missing/have different structures due to software updates. 
> Note: Most common differences are found in 'move_group.launch', 'moveit_planning_execution.launch' and 'trajectory_execution.launch.xml'. Check all namespaces match this repository

## Test the changes using the following steps 
1. Use the fanuc tutorials to ensure that connection with the robot is established and the required drivers are running. 
2. Ensure the moveit_config works standalone, before testing fc_launch. Use the following commands: 
```shell
roslaunch fanuc_lrmate200id7l_moveit_config moveit_planning_execution.launch sim:=true 
roslaunch fanuc_lrmate200id7l_moveit_config moveit_planning_execution.launch sim:=false robot_ip:=<robot_ip>
```
3. Check fc_tasks runs in simulation by keeping sim:=true with no namespace. 
```shell
roslaunch fc_launch moveit.launch sim:=true namespace:=''
```
4. Test with a namespace.
```shell
roslaunch fc_launch moveit.launch sim:=true namespace:='sim1'
```
5. Test on a real robot. 
```shell
roslaunch fc_launch moveit.launch sim:=false namespace:='real'
```
6. Check 'plan and execute' works in RViz by setting a 'random valid' goal. 
7. Check all ROS services and actions work. An example is shown below:
```shell
rosservice call /sim1/fc_get_pose sim1/base_link 
```

> Note: This tutorial can be adapted to non-Fanuc robot arms. However, problems may occur due to version difference of support and moveit configs. It is recommended to use this interface repository as a reference and build the new package from the beginning. 