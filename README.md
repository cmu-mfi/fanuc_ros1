# fanuc_ros1

A ROS-Industrial interface for the Fanuc LRMate200iD7L robot arm.

## Pre-requisites

1. A Fanuc Robot Arm
2. ROS-Industial drivers installed on the Fanuc Controller. (Follow instructions here: http://wiki.ros.org/fanuc/Tutorials)
3. A Linux PC connected to the controller via Ethernet

## Installation

**Pre-requisite**

```shell
# Check if docker is installed
$ docker version
Client: Docker Engine - Community
 Version:           20.10.7
...

# Check if docker is running
$ systemctl is-active docker
active
```

- If installation is needed, install docker enginer from here: \
  [https://docs.docker.com/engine/install/](https://docs.docker.com/engine/install/)
- If docker is not running, followin troubleshooting steps: [https://docs.docker.com/engine/daemon/troubleshoot/](https://docs.docker.com/engine/daemon/troubleshoot/)

**Build and Run**

> Note: Check ROS_MASTER_URI in `compose.yml`. If not running roscore on host, comment out the line.

```shell
docker compose up --build
```

The above command will build and instantiate the built image as a container 'fanucros-container'.
Use the following commands in a new terminal to attach to the terminal with X-forwarding enabled to visualize GUIs (RViz) launched inside the docker.

```shell
xhost local:docker
docker exec -it fanucros-container bash
```

To keep any changes made inside the container, use the following command to re-start the same docker container on next usage.

```shell
docker start -i fanucros-container
```

## System Bringup Instructions

Build the workspace and source the setup file

```shell
cd ros1_ws/
catkin build
source devel/setup.bash
```

### ROS Launch Command

'moveit.launch': Connects with a robot. Also starts moveGroup interface and fc_tasks ROS service interface. More info on fc_tasks available in the package's README.

> Note: Change the ip address in 'fc_launch/launch/moveit.launch' file to the real robot ip. Ignore if running in simulation.

```
roslaunch fc_launch moveit.launch namespace:=<namespace> sim:=<true/false>
```

- 'sim': Default value is 'false'. To run in simulation, specify 'true'.
- 'namespace': Launches robot in a specified namespace.

Default supported namespaces are: 'sim1', 'sim2', '', 'real'. To add/remove a namespace, add the corresponding controller name in '<robot_name>\_moveit_config/config/controllers'

> Note: To speed up system bringup, comment out all unused controllers. The system will try to find all provided controllers and won't allow planning until either all are found or timed out. Timed out controller servers do not affect system performance.

## Important Assets

### Support Packages for Fanuc

ROS support packages are [provided by fanuc](https://github.com/ros-industrial/fanuc).
Tutorials and documentation for ROS1 from Fanuc available here: [Fanuc Wiki](http://wiki.ros.org/fanuc/Tutorials)

### LRMate200iD7L MoveIt Configuration

This repository contains the moveit configuration files for the LRMate200iD7L in the 'fanuc_lrmate200id7l_moveit_config' package. It depends on the 'fanuc_lrmate200id_package'

- Execute RViz Simulation

```shell
roslaunch fanuc_lrmate200id7l_moveit_config moveit_planning_execution.launch sim:=true
```

- Execute MoveIt for the real robot

```shell
roslaunch fanuc_lrmate200id7l_moveit_config moveit_planning_execution.launch sim:=false robot_ip:=<robot_ip>
```

> Note: As industrial controller take longer to execute trajectories than MoveIt expects, you may need to increase the parameters `allowed_execution_duration_scaling` and `allowed_goal_duration_margin`. If that doesn't work, comment out the functionality from moveit_core
