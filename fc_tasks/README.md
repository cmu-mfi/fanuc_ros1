# fc_tasks

ROS-Industrial interface for Fanuc Robot Arms

## FC Interface Class

Initializes the MoveIt! MoveGroup interface and sets up all relevant ROS interfaces including action servers, service servers, and topic subscribers for controlling a robotic manipulator.

Interface Name = move*group*\
Group Name = manipulator\
Default Planning Pipeline = pilz_industrial_motion_planner

> Note: All below ROS elements will be prefixed with the chosen namespace. (Ex: /sim1/fc_get_pose)

### ROS Services

- getPose - <i>'/fc_get_pose'</i>
- setPose - <i>'/fc_set_pose'</i>
- setJoints - <i>'/fc_set_joints'</i> (Uses BiTRRT)
- executeTrajectory - <i>'/fc_execute_trajectory'</i>
- stopTrajectory - <i>'/fc_stop_trajectory' </i>
- executeCartesianTrajectory - <i>'/fc_execute_cartesian_trajectory'</i>
- executeCartesianTrajectoryAsync - <i>'/fc_execute_cartesian_trajectory_async'</i>

### ROS Actions

- GoToPose - <i>'/fc_go_to_pose'</i>
- GoToPose (Async) - <i>'/fc_go_to_pose_async'</i>
- GoToJoints - <i>'/fc_go_to_joints'</i> (Uses BiTRRT)
- ExecuteCartesianTrajectory - <i>'/fc_execute_cartesian_trajectory_action'</i>

### ROS Topic Subscribers

- CheckMoving - <i>'/check_moving'</i>
- TrajectoryStatus - <i>'/execute_trajectory/status'</i>
- JointStates - <i>'/joint_states'</i>

### ROS Topic Publishers

- End Effector Pose - <i>'/tool0_pose'</i>

> Note: All methods called by these elements are detailed in the Doxygen formatting style.

## Dependencies

- fc_launch
- fc_msgs
- fanuc_lrmate200id_support
- fanuc_lrmate200id7l_moveit_config
- fanuc

## Fanuc IO Interface

This package also enables Fanuc I/O control using [comet_rpc](https://github.com/gavanderhoorn/comet_rpc). Using the io.launch file, a Fanuc_IO ROS node is launched with the following functionality -

### ROS Services

- setIOValue - <i>/set_io_value</i>
- readIOValue - <i>/read_io_value</i>

Note: These services use the SetIO.srv and ReadIO.srv messages from fc_msgs. Please use rosservice info to get all input details.

### ROS Publishers

- <i>/io_states_DOUT</i> - Publishes all Digital Out I/O states
- <i>/io_states_DIN</i> - Publishes all Digital In I/O states
- <i>/io_states_AOUT</i> - Publishes all Analog Out I/O states
- <i>/io_states_AIN</i> - Publishes all Analog In I/O states
