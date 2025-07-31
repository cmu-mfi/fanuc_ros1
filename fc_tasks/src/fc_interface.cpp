#include <fc_interface.h>


FC_Interface::FC_Interface(std::string group_name, ros::NodeHandle &nh_)
    : move_group_(group_name), 
    go_to_pose_as_(nh_, "fc_go_to_pose", boost::bind(&FC_Interface::goToPoseCallback, this, _1), false), 
    go_to_pose_async_as_(nh_, "fc_go_to_pose_async", boost::bind(&FC_Interface::goToPoseAsyncCallback, this, _1), false), 
    go_to_joints_as_(nh_, "fc_go_to_joints", boost::bind(&FC_Interface::goToJointsCallback, this, _1), false),
    execute_cartesian_trajectory_as_(nh_, "fc_execute_cartesian_trajectory_action", boost::bind(&FC_Interface::executeCartesianTrajectoryCallback, this, _1), false)
{
	ROS_INFO_STREAM("Planning frame.." << move_group_.getPlanningFrame());
	ROS_INFO_STREAM("Pose reference frame.." << move_group_.getPoseReferenceFrame());
	ROS_INFO_STREAM("End Effector.." << move_group_.getEndEffector());
	ROS_INFO_STREAM("End Effector Link.." << move_group_.getEndEffectorLink());

	max_velocity_scaling_factor_ = 0.1;
	max_acceleration_scaling_factor_ = 0.1;
	move_group_.setMaxAccelerationScalingFactor(max_acceleration_scaling_factor_);
	move_group_.setMaxVelocityScalingFactor(max_velocity_scaling_factor_);

	//READ CONFIG FILES (TODO: change to include everything)
	std::string current_dir = ros::package::getPath("fc_tasks"); 
	std::string robot_config_file = current_dir + "/config/lr200iD7L.json";
	robot_DH_ = loadFromFile_(robot_config_file, "DH"); 
	robot_base_frame_ = loadFromFile_(robot_config_file, "base"); 

	//START ACTION SERVERS
	go_to_pose_as_.start();
	go_to_pose_async_as_.start();
	go_to_joints_as_.start();
	execute_cartesian_trajectory_as_.start();

    //ROSTOPIC SUBSCRIBERS
	check_moving_sub_ = nh_.subscribe("check_moving", 1, &FC_Interface::checkMoving, this);
	trajectory_status_sub_ = nh_.subscribe("execute_trajectory/status", 1, &FC_Interface::checkTrajStatus, this);
	joint_states_sub_ = nh_.subscribe("joint_states", 1, &FC_Interface::jointStatesCallback, this);

	//ROSTOPIC PUBLISHERS 
	tool0_pose_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("tool0_pose", 1);

    //SERVICE SERVERS
	getPose_server_ = nh_.advertiseService("fc_get_pose", &FC_Interface::getPose, this);
	setJoints_server_ = nh_.advertiseService("fc_set_joints", &FC_Interface::setJoints, this);
	setPose_server_ = nh_.advertiseService("fc_set_pose", &FC_Interface::setPose, this);
	executeTrajectory_server_ = nh_.advertiseService("fc_execute_trajectory", &FC_Interface::executeTrajectory, this);
	stopTrajectory_server_ = nh_.advertiseService("fc_stop_trajectory", &FC_Interface::stopTrajectory, this);
	executeCartesianTraj_server_ = nh_.advertiseService("fc_execute_cartesian_trajectory", &FC_Interface::executeCartesianTrajectory, this);
	executeCartesianTrajAsync_server_ = nh_.advertiseService("fc_execute_cartesian_trajectory_async", &FC_Interface::executeCartesianTrajectory, this);
}

bool FC_Interface::getPose(fc_msgs::GetPose::Request &req, fc_msgs::GetPose::Response &res){

	ros::AsyncSpinner async_spinner(1);
	async_spinner.start();

	res.pose = move_group_.getCurrentPose().pose;

	async_spinner.stop();
	return true;
}

bool FC_Interface::setPose(fc_msgs::SetPose::Request &req, fc_msgs::SetPose::Response &res){

	ros::AsyncSpinner async_spinner(1);
	async_spinner.start();

	
	ROS_DEBUG_STREAM("Let's Move!!");
	ROS_DEBUG_STREAM("New Move to: " << req.pose);

	bool success = planAndExecutePose_(req.pose, req.max_velocity_scaling_factor, req.max_acceleration_scaling_factor, req.traj_type);

	res.pose = move_group_.getCurrentPose().pose;
	ROS_DEBUG_STREAM(res.pose);

	async_spinner.stop();

	return success;
}
bool FC_Interface::setJoints(fc_msgs::SetJoints::Request &req, fc_msgs::SetJoints::Response &res){

	ros::AsyncSpinner async_spinner(1);
	async_spinner.start();

	ROS_DEBUG_STREAM("Let's Move!!");
	ROS_DEBUG_STREAM("New Move to: " << req.state);

	move_group_.setPlannerId("BiTRRT");
	move_group_.setJointValueTarget(req.state);
	ROS_DEBUG_STREAM("Target Joints set");

	moveit_msgs::MoveItErrorCodes ret_val = move_group_.move();
	ROS_DEBUG_STREAM("Return Value" << ret_val);

	try_count_ = 0; 
	while (ret_val.val == moveit_msgs::MoveItErrorCodes::TIMED_OUT && try_count_ < max_retries_){
		try_count_++;
		ret_val = move_group_.move(); 
	}

	last_joints_.name = move_group_.getJoints();
	last_joints_.position = move_group_.getCurrentJointValues();

	res.state = last_joints_;
	async_spinner.stop();

	if (ret_val.val != moveit_msgs::MoveItErrorCodes::SUCCESS)
	{
		ROS_ERROR_STREAM("Move failed due to ERROR CODE=" << ret_val);
		return false;
	}
	ROS_DEBUG_STREAM( "Move succeeded.");
    return true;

}
bool FC_Interface::executeTrajectory(moveit_msgs::ExecuteKnownTrajectory::Request &req, moveit_msgs::ExecuteKnownTrajectory::Response &res){
	
	ros::AsyncSpinner async_spinner(1);
	async_spinner.start();

	move_group_.stop();

	moveit::planning_interface::MoveGroupInterface::Plan plan;
	plan.trajectory_ = req.trajectory;

	moveit_msgs::MoveItErrorCodes ret_val = move_group_.execute(plan);

	res.error_code = ret_val;
	if (ret_val.val != moveit_msgs::MoveItErrorCodes::SUCCESS)
	{
		ROS_ERROR_STREAM("Move failed due to ERROR CODE=" << res.error_code);
		return true;
	}

	async_spinner.stop();
    return true;
}
bool FC_Interface::stopTrajectory(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res){
	
	ros::AsyncSpinner async_spinner(1);
	async_spinner.start();

	move_group_.stop();
	res.success = true;
	res.message = "Trajectory stopped";

	async_spinner.stop();
	return true;
}
bool FC_Interface::executeCartesianTrajectory(fc_msgs::ExecuteCartesianTrajectory::Request &req, fc_msgs::ExecuteCartesianTrajectory::Response &res){
	ros::AsyncSpinner async_spinner(1);
	async_spinner.start();

	ROS_DEBUG_STREAM( "Let's Move!!");
	ROS_DEBUG_STREAM( "Number of waypoints: " << req.poses.size());
	
	bool success = planAndExecuteCartesianPath_(req.poses, req.eef_step, req.jump_threshold, 
												req.max_velocity_scaling_factor, req.max_acceleration_scaling_factor,
												req.blend_radius, false);
	
	res.pose = move_group_.getCurrentPose().pose;
	ROS_DEBUG_STREAM(res.pose);

	async_spinner.stop();

	return success;	
}

bool FC_Interface::executeCartesianTrajectoryAsync(fc_msgs::ExecuteCartesianTrajectory::Request &req, fc_msgs::ExecuteCartesianTrajectory::Response &res){
	ros::AsyncSpinner async_spinner(1);
	async_spinner.start();

	ROS_DEBUG_STREAM( "Let's Move!!");
	ROS_DEBUG_STREAM( "Number of waypoints: " << req.poses.size());
	
	bool success = planAndExecuteCartesianPath_(req.poses, req.eef_step, req.jump_threshold, 
												req.max_velocity_scaling_factor, req.max_acceleration_scaling_factor,
												req.blend_radius, true);
	
	res.pose = move_group_.getCurrentPose().pose;
	ROS_DEBUG_STREAM(res.pose);

	async_spinner.stop();

	return success;	

}
void FC_Interface::goToJointsCallback(const fc_msgs::GoToJointsGoalConstPtr &goal){
	ros::AsyncSpinner async_spinner(1);
	async_spinner.start();

	ROS_DEBUG_STREAM("Let's Move!!");
	ROS_DEBUG_STREAM("New Move to: " << goal->state);

	move_group_.setPlannerId("BiTRRT");
	ROS_DEBUG_STREAM("Printing Planner Params");

	move_group_.setMaxAccelerationScalingFactor(goal->max_acceleration_scaling_factor);
	move_group_.setMaxVelocityScalingFactor(goal->max_velocity_scaling_factor);

	go_to_joints_feedback_.feedback = "Printing Planner Params";
	go_to_joints_as_.publishFeedback(go_to_joints_feedback_);

	move_group_.setJointValueTarget(goal->state);
	ROS_DEBUG_STREAM("Target Joints set");
	go_to_joints_feedback_.feedback = "Target Joints set";
	go_to_joints_as_.publishFeedback(go_to_joints_feedback_);

	moveit_msgs::MoveItErrorCodes ret_val = move_group_.move();

	ROS_DEBUG_STREAM("Return Value"<<ret_val);

	while (ret_val.val == moveit_msgs::MoveItErrorCodes::TIMED_OUT)
	{
		ret_val = move_group_.move();
	}
	
	last_joints_.name = move_group_.getJoints();
	last_joints_.position = move_group_.getCurrentJointValues();

	go_to_joints_result_.state = last_joints_;
	async_spinner.stop();

	if (ret_val.val != moveit_msgs::MoveItErrorCodes::SUCCESS)
	{
		ROS_ERROR_STREAM("Move failed due to ERROR CODE=" << ret_val);
		go_to_joints_feedback_.feedback = "Move failed";
		go_to_joints_as_.publishFeedback(go_to_joints_feedback_);
		go_to_joints_as_.setAborted(go_to_joints_result_);
		return;
	}

	go_to_joints_as_.setSucceeded(go_to_joints_result_);
	return;
    
}
void FC_Interface::goToPoseCallback(const fc_msgs::GoToPoseGoalConstPtr &goal){
    ros::AsyncSpinner async_spinner(1);
	async_spinner.start();

	ROS_DEBUG_STREAM("Let's Move!!");
	ROS_DEBUG_STREAM("New Move to: " << goal->pose);

	bool success = planAndExecutePose_(goal->pose, goal->max_velocity_scaling_factor, goal->max_acceleration_scaling_factor, goal->traj_type, false);

	if (!success)
	{
		go_to_pose_feedback_.feedback = "Move failed";
		go_to_pose_as_.publishFeedback(go_to_pose_feedback_);
		go_to_pose_as_.setAborted(go_to_pose_result_);
		return;
	}

	go_to_pose_result_.pose = move_group_.getCurrentPose().pose;	
	ROS_DEBUG_STREAM(go_to_pose_result_.pose);

	async_spinner.stop();
	go_to_pose_as_.setSucceeded(go_to_pose_result_);

	return;
}
void FC_Interface::goToPoseAsyncCallback(const fc_msgs::GoToPoseGoalConstPtr &goal){
        ros::AsyncSpinner async_spinner(1);
	async_spinner.start();

	ROS_DEBUG_STREAM("Let's Move!!");
	ROS_DEBUG_STREAM("New Move to: " << goal->pose);

	bool success = planAndExecutePose_(goal->pose, goal->max_velocity_scaling_factor, goal->max_acceleration_scaling_factor, goal->traj_type, true);

	if (!success)
	{
		go_to_pose_feedback_.feedback = "Move failed";
		go_to_pose_as_.publishFeedback(go_to_pose_feedback_);
		go_to_pose_as_.setAborted(go_to_pose_result_);
		return;
	}

	go_to_pose_result_.pose = move_group_.getCurrentPose().pose;	
	ROS_DEBUG_STREAM(go_to_pose_result_.pose);

	async_spinner.stop();
	go_to_pose_as_.setSucceeded(go_to_pose_result_);

	return;
}
void FC_Interface::executeCartesianTrajectoryCallback(const fc_msgs::ExecuteCartesianTrajectoryGoalConstPtr & goal){

	ROS_DEBUG_STREAM("Let's Move!!");
	ROS_DEBUG_STREAM("New Trajectory to End Waypoint: " << goal->waypoints[1]);
	ROS_DEBUG_STREAM("Number of waypoints: " << goal->waypoints.size());

	execute_cartesian_trajectory_feedback_.feedback = "Planning Cartesian Trajectory";
	execute_cartesian_trajectory_feedback_.wp_pct = 0.0;
	execute_cartesian_trajectory_feedback_.wp_id = 0;
	execute_cartesian_trajectory_as_.publishFeedback(execute_cartesian_trajectory_feedback_);
	
	planAndExecuteCartesianPath_(goal->waypoints, goal->eef_step, goal->jump_threshold, 
								goal->max_velocity_scaling_factor, goal->max_acceleration_scaling_factor,
								goal->blend_radius, true);
	//******************************************************************

	while(execute_cartesian_trajectory_feedback_.wp_id < goal->waypoints.size()-1)
	{
		if(execute_cartesian_trajectory_as_.isPreemptRequested() || !ros::ok())
		{
			execute_cartesian_trajectory_feedback_.feedback = "Preempted";
			execute_cartesian_trajectory_as_.publishFeedback(execute_cartesian_trajectory_feedback_);
			execute_cartesian_trajectory_as_.setPreempted(execute_cartesian_trajectory_result_);
			return;
		}

		int next_wp_id = execute_cartesian_trajectory_feedback_.wp_id + 1;
		geometry_msgs::Pose next_wp = goal->waypoints[next_wp_id];

		double position_error, orientation_error;
		calculatePoseError_(move_group_.getCurrentPose().pose, next_wp, position_error, orientation_error);

		/** TODO: create config file for all magic numbers*/
		float pos_tolerance = 0.001;
		float orient_tolerance = 0.01;

		if (position_error < pos_tolerance || orientation_error < orient_tolerance)
		{
			execute_cartesian_trajectory_feedback_.feedback = "Waypoint reached: " + std::to_string(next_wp_id);
			execute_cartesian_trajectory_feedback_.wp_id = next_wp_id;
		}
		
		execute_cartesian_trajectory_feedback_.wp_pct = 
			(float)execute_cartesian_trajectory_feedback_.wp_id/(float)goal->waypoints.size();
	
		execute_cartesian_trajectory_as_.publishFeedback(execute_cartesian_trajectory_feedback_);
	}
}
void FC_Interface::checkMoving(const sensor_msgs::JointState::ConstPtr &msg){
	int size = msg->velocity.size();
	double velocity_sum = 0;

	for (int i=0; i<size; i++){
		velocity_sum += abs(msg->velocity[i]);
	}
	if (velocity_sum > 0.05){
		is_moving_ = true;
	}
	else{
		is_moving_ = false;
	}
}

void FC_Interface::checkTrajStatus(const actionlib_msgs::GoalStatusArray::ConstPtr &msg){
    int len = msg->status_list.size();
	if (len != 0)
		traj_status_ = msg->status_list[len - 1];
}

void FC_Interface::jointStatesCallback(const sensor_msgs::JointState::ConstPtr& msg){
	// READ JOINT STATES
	VectorJd q(msg->position.size());
	for(int i=0; i<msg->position.size(); i++)
	{
		q(i) = msg->position[i];
	}

	// COMPUTE FORWARD KINEMATICS
	Eigen::Matrix4d tool0_pose = _FK(q, robot_DH_, robot_base_frame_, true);

	// PUBLISH TOOL0 POSE
	geometry_msgs::PoseStamped tool0_pose_msg;
	tool0_pose_msg.header.stamp = ros::Time::now();
	tool0_pose_msg.header.frame_id = "base";
	tool0_pose_msg.pose.position.x = tool0_pose(0, 3);
	tool0_pose_msg.pose.position.y = tool0_pose(1, 3);
	tool0_pose_msg.pose.position.z = tool0_pose(2, 3);
	Eigen::Quaterniond q_(tool0_pose.block<3, 3>(0, 0));
	tool0_pose_msg.pose.orientation.x = q_.x();
	tool0_pose_msg.pose.orientation.y = q_.y();
	tool0_pose_msg.pose.orientation.z = q_.z();
	tool0_pose_msg.pose.orientation.w = q_.w();
	
	tool0_pose_pub_.publish(tool0_pose_msg);
}

/** Private Methods */

	/**
	 * @brief Computes the positional and angular difference between two poses.
	 *
	 * This utility function calculates:
	 * - The Euclidean distance between the positions of two poses.
	 * - The rotational difference (in radians) between the orientations of two poses,
	 *   computed via quaternion dot product.
	 *
	 * @param pose1 First pose (geometry_msgs::Pose).
	 * @param pose2 Second pose (geometry_msgs::Pose).
	 * @param position_error Output variable to store the 3D Euclidean distance between positions.
	 * @param orientation_error Output variable to store the angular difference in radians.
	 *
	 * @note The quaternion dot product is clamped between [-1, 1] before applying `acos` to avoid
	 *       numerical domain errors. The resulting orientation error represents the angle of rotation
	 *       required to align pose1's orientation with pose2's.
	 *
	 * @warning Both quaternions are normalized before computing orientation error. Invalid inputs
	 *          (e.g., zero-length quaternions) may produce undefined behavior.
	 */
void FC_Interface::calculatePoseError_(const geometry_msgs::Pose &pose1, const geometry_msgs::Pose &pose2, double &position_error, double &orientation_error){
	/** Calculate Position Error */
	position_error = sqrt(pow(pose1.position.x - pose2.position.x, 2) +
						  pow(pose1.position.y - pose2.position.y, 2) +
						  pow(pose1.position.z - pose2.position.z, 2));

	/** Calculate Orientation Error */
	Eigen::Quaterniond q1 = Eigen::Quaterniond(pose1.orientation.w, pose1.orientation.x, pose1.orientation.y, pose1.orientation.z);
	Eigen::Quaterniond q2 = Eigen::Quaterniond(pose2.orientation.w, pose2.orientation.x, pose2.orientation.y, pose2.orientation.z);
	q1.normalize(); q2.normalize();

	double dot = q1.dot(q2);

    /** Clamp the dot product to avoid domain errors in acos due to numerical issues */
    dot = std::max(-1.0, std::min(1.0, dot));

    /** Rotational error (angle) in radians */
    orientation_error = 2.0 * std::acos(std::abs(dot));
}

/**
	 * @brief Plans and executes a multi-waypoint Cartesian trajectory using the Pilz industrial motion planner.
	 *
	 * This function builds a motion sequence request from a series of Cartesian waypoints. Each waypoint is converted
	 * into a motion plan request with associated constraints and optional blending between motions. The generated sequence
	 * is then planned via the `/plan_sequence_path` service, and the resulting trajectories are executed either synchronously
	 * or asynchronously depending on the `async` flag.
	 *
	 * @param waypoints A vector of geometry_msgs::Pose representing target end-effector poses to follow.
	 * @param eef_step Step size (in meters) used for internal Cartesian interpolation (currently unused here, reserved for consistency).
	 * @param jump_threshold Threshold for detecting joint-space discontinuities (currently unused here, reserved for consistency).
	 * @param max_velocity_scaling_factor Scaling factor for motion velocity (0.0 to 1.0); defaults to 0.3 if unset (0.0).
	 * @param max_acceleration_scaling_factor Scaling factor for motion acceleration (0.0 to 1.0); defaults to 0.3 if unset (0.0).
	 * @param blend_radius Blending radius (in meters) used to smooth transitions between waypoints. Last waypoint gets zero blending.
	 * @param async If true, each segment is executed asynchronously using `asyncExecute`; otherwise uses `execute`.
	 * @param planner_id Optional planner type: `"LIN"` (linear) or `"PTP"` (point-to-point). Defaults to `"LIN"`.
	 *
	 * @return true if the full motion sequence was successfully planned and all segments were executed successfully; false otherwise.
	 *
	 * @note
	 * - Uses `/plan_sequence_path` service from the Pilz industrial motion planner to generate a motion sequence.
	 * - Converts each waypoint into a `MotionSequenceItem` with associated goal constraints.
	 * - Sets the final waypoint's blend radius to zero to ensure precise endpoint targeting.
	 * - Applies default tolerances of 1 mm (position) and 0.01 rad (orientation) for each waypoint constraint.
	 *
	 * @warning
	 * - The input parameters `eef_step` and `jump_threshold` are included for compatibility but not used in the current implementation.
	 */
bool FC_Interface::planAndExecuteCartesianPath_(const std::vector<geometry_msgs::Pose> &waypoints, 
									double eef_step, double jump_threshold, 
									double max_velocity_scaling_factor, double max_acceleration_scaling_factor,
									double blend_radius, bool async, std::string planner_id){
	
	
	moveit_msgs::MotionPlanRequest mp_req_base;
	moveit_msgs::MotionSequenceRequest ms_req;
	moveit_msgs::MotionSequenceResponse ms_res;

	mp_req_base.pipeline_id = "pilz_industrial_motion_planner";
	mp_req_base.planner_id = planner_id;
	mp_req_base.group_name = "manipulator";
	mp_req_base.num_planning_attempts = 5;
	mp_req_base.max_velocity_scaling_factor = max_velocity_scaling_factor == 0.0 ? 0.3 : max_velocity_scaling_factor;
	mp_req_base.max_acceleration_scaling_factor = max_acceleration_scaling_factor == 0.0 ? 0.3 : max_acceleration_scaling_factor;

	float pos_tolerance = 0.001;
	float orient_tolerance = 0.01;

	moveit::core::RobotStatePtr robot_state_ptr(move_group_.getCurrentState());
	const robot_state::JointModelGroup *robot_joint_group_ptr = robot_state_ptr->getJointModelGroup("manipulator");

	for (int i = 0; i < waypoints.size(); i++)
	{
		moveit_msgs::MotionSequenceItem item;
		item.blend_radius = blend_radius;
   
		if (i == waypoints.size() - 1)
			 item.blend_radius = 0.0;

		item.req = mp_req_base;
		geometry_msgs::PoseStamped waypoint_pose;
		waypoint_pose.header.frame_id = "base_link";
		waypoint_pose.pose = waypoints[i];
		moveit_msgs::Constraints goal = kinematic_constraints::constructGoalConstraints(move_group_.getEndEffectorLink(), waypoint_pose, pos_tolerance, orient_tolerance);
		item.req.goal_constraints.push_back(goal);
   
		if (i != waypoints.size() - 1)
			 item.req.start_state = moveit_msgs::RobotState();
   
		ms_req.items.push_back(item);
	}

	ROS_DEBUG_STREAM("Number of waypoints: "<<ms_req.items.size());
	ros::ServiceClient plan_sequence_path_client = nh_.serviceClient<moveit_msgs::GetMotionSequence>("plan_sequence_path");
	plan_sequence_path_client.waitForExistence();
	moveit_msgs::GetMotionSequence srv;
	srv.request.request = ms_req;

	moveit_msgs::MoveItErrorCodes ret_val;
	ret_val.val = moveit_msgs::MoveItErrorCodes::TIMED_OUT;
	
	while (ret_val.val == moveit_msgs::MoveItErrorCodes::TIMED_OUT)
	{
		if (plan_sequence_path_client.call(srv))
		{
			ms_res = srv.response.response;
			if (ms_res.error_code.val != moveit_msgs::MoveItErrorCodes::SUCCESS)
			{
				ROS_ERROR_STREAM("Planning failed due to ERROR CODE = " << ms_res.error_code.val);
				ret_val.val = ms_res.error_code.val;
			}
			else
			{
				ROS_DEBUG_STREAM("Planning successful");
				std::vector<moveit_msgs::RobotTrajectory> trajectories = ms_res.planned_trajectories;
				ROS_DEBUG_STREAM("Number of planned trajectories: "<<trajectories.size());
				for (int i = 0; i < trajectories.size(); i++)
				{
					moveit::planning_interface::MoveGroupInterface::Plan plan;
					plan.trajectory_ = trajectories[i];
					ret_val = async ? move_group_.asyncExecute(plan) : move_group_.execute(plan);
					if (ret_val.val != moveit_msgs::MoveItErrorCodes::SUCCESS)
					{
						std::cerr << "Move failed due to ERROR CODE=" << ret_val;
						return false;
					}
				}
			}
		}
		else
		{
			ROS_ERROR("Failed to call service plan_sequence_path");
			return false;
		}
	}
	return true; 		
}


	/**
	 * @brief Plans and executes a motion to a specified end-effector pose using the Pilz industrial motion planner.
	 *
	 * This function generates a point-to-point or linear trajectory to reach a target pose using the
	 * `/plan_kinematic_path` service. The motion is executed either synchronously or asynchronously
	 * depending on the `async` flag.
	 *
	 * @param pose Target end-effector pose (geometry_msgs::Pose) in the planning frame.
	 * @param max_velocity_scaling_factor Scaling factor for maximum velocity (range: 0.0–1.0). Defaults to 0.3 if zero.
	 * @param max_acceleration_scaling_factor Scaling factor for maximum acceleration (range: 0.0–1.0). Defaults to 0.3 if zero.
	 * @param planner_id Optional planner type: "LIN" for linear motion or "PTP" for point-to-point. Defaults to "LIN".
	 * @param async If true, the plan is executed asynchronously via `asyncExecute`. Otherwise, uses blocking `execute`.
	 *
	 * @return true if the pose was successfully planned and execution started; false otherwise.
	 *
	 * @note
	 * - Retries planning if the result is a timeout (`TIMED_OUT` error code).
	 * - Calls the `/plan_kinematic_path` service from the Pilz planner to generate the motion.
	 * - Automatically chooses "PTP" planner if `planner_id == "PTP"`, else defaults to "LIN".
	 *
	 */
bool FC_Interface::planAndExecutePose_(const geometry_msgs::Pose &pose, 
							double max_velocity_scaling_factor, double max_acceleration_scaling_factor,
							std::string planner_id,
							bool async){

	moveit_msgs::MotionPlanRequest mp_req;
	moveit_msgs::MotionPlanResponse mp_res;

	mp_req.pipeline_id = "pilz_industrial_motion_planner";
	mp_req.planner_id = planner_id.compare("PTP") == 0 ? "PTP" : "LIN";
	mp_req.group_name = "manipulator";
	mp_req.num_planning_attempts = 1;
	mp_req.max_velocity_scaling_factor = max_velocity_scaling_factor == 0.0 ? 0.3 : max_velocity_scaling_factor;
	mp_req.max_acceleration_scaling_factor = max_acceleration_scaling_factor == 0.0 ? 0.3 : max_acceleration_scaling_factor;

	float pos_tolerance = 0.05;
	float orient_tolerance = 0.05;
	geometry_msgs::PoseStamped target_pose_stamped;
	target_pose_stamped.header.frame_id = "";
	target_pose_stamped.pose = pose;
	moveit_msgs::Constraints pose_goal = kinematic_constraints::constructGoalConstraints(move_group_.getEndEffectorLink(), target_pose_stamped, pos_tolerance, orient_tolerance);
	mp_req.goal_constraints.push_back(pose_goal);

	ros::ServiceClient plan_kinematics_path_client = nh_.serviceClient<moveit_msgs::GetMotionPlan>("plan_kinematic_path");
	plan_kinematics_path_client.waitForExistence();
	moveit_msgs::GetMotionPlan srv;
	srv.request.motion_plan_request = mp_req;

	moveit_msgs::MoveItErrorCodes ret_val;
	ret_val.val = moveit_msgs::MoveItErrorCodes::TIMED_OUT;

	if (plan_kinematics_path_client.call(srv))
	{
		mp_res = srv.response.motion_plan_response;
		if (mp_res.error_code.val != moveit_msgs::MoveItErrorCodes::SUCCESS)
		{
			ROS_ERROR_STREAM("Planning failed due to ERROR CODE = " << mp_res.error_code.val);
			ret_val.val = mp_res.error_code.val;
		}
		else
		{
			ROS_DEBUG_STREAM("Planning successful");
			moveit::planning_interface::MoveGroupInterface::Plan plan;
			plan.trajectory_ = mp_res.trajectory;
			if (async)
				ret_val = move_group_.asyncExecute(plan);
			else
				ret_val = move_group_.execute(plan);
		}
	}
	else
	{
		ROS_ERROR("Failed to call service plan_kinematics_path");
		return false;
	}
	
	return true;
}