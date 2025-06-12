# Tutorial 2 - Using scripts to test actions and services 

This repository comes with example scripts to test actions and services in 'fc_tasks/scripts'. These are outlined below :

### SetPose Service Test
The setPose_test.py script contains an example pose to test the /fc_set_pose service. Modify this as required to test. It's a good idea to execute the same pose multiple times to ensure that all frames are calibrated correctly. 
```shell
python3 ~/ros1_ws/src/fc_tasks/scripts/setPose_test.py
```
### ExecuteCartesianTrajectory Service Test
The cartTraj_test.py script contains a trajectory with 2 waypoints to test the /fc_execute_cartesian_trajectory service. Modify this as required to test. 
```shell
python3 ~/ros1_ws/src/fc_tasks/scripts/cartTraj_test.py
```
### GoToPose Action Test 
The goToPoseAction_test.py script contains an example pose to test the /fc_go_to_pose action server. 
```shell
python3 ~/ros1_ws/src/fc_tasks/scripts/goToPoseAction_test.py
```
### ExecuteCartesianTrajectory Action Test 
The execTrajAction_test.py contains a trajectory with 2 waypoints to test the /fc_execute_cartesian_trajectory_action action server. 
```shell
python3 ~/ros1_ws/src/fc_tasks/scripts/execTrajAction_test.py