#ifndef _FC_INTERFACE_
#define _FC_INTERFACE_

#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <ros/package.h>
#include <fc_msgs/GoToPoseAction.h>
#include <fc_msgs/GoToJointsAction.h>
#include <fc_msgs/ExecuteCartesianTrajectoryAction.h>
#include <fc_msgs/GetPose.h>
#include <fc_msgs/GetPoseStamped.h>
#include <fc_msgs/SetPose.h>
#include <fc_msgs/SetJoints.h>
#include <fc_msgs/ExecuteCartesianTrajectory.h>
#include <tf/transform_listener.h>
#include <tf/tf.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_interface/planning_interface.h>
#include <moveit_msgs/MoveItErrorCodes.h>
#include <moveit_msgs/RobotTrajectory.h>
#include <moveit_msgs/GetMotionPlan.h>
#include <moveit_msgs/MotionPlanRequest.h>
#include <moveit_msgs/MotionPlanResponse.h>
#include <moveit_msgs/GetMotionSequence.h>
#include <moveit_msgs/MotionSequenceRequest.h>
#include <moveit_msgs/MotionSequenceResponse.h>
#include <moveit_msgs/ExecuteKnownTrajectory.h>
#include <moveit/kinematic_constraints/utils.h>
#include <geometry_msgs/WrenchStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Quaternion.h>
#include <industrial_msgs/RobotStatus.h>
#include <std_srvs/Trigger.h>
#include <string.h>
#include <cmath>
#include <Eigen/Geometry>
#include <actionlib_msgs/GoalStatusArray.h>
#include <actionlib_msgs/GoalStatus.h>
#include <iostream> 
#include <fstream> 
#include <sensor_msgs/JointState.h>
#include <Eigen/Dense>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <jsoncpp/json/json.h>
#include <jsoncpp/json/value.h>

typedef Eigen::Matrix<double, Eigen::Dynamic, 1> VectorJd;
#define PI 3.141592653589793

class FC_Interface{
private:
	ros::NodeHandle nh_;

    // ROS SERVICE SERVERS
	ros::ServiceServer getPose_server_;
	ros::ServiceServer getPoseStamped_server_;
	ros::ServiceServer setPose_server_;
	ros::ServiceServer setJoints_server_;
	ros::ServiceServer executeTrajectory_server_;
	ros::ServiceServer stopTrajectory_server_;
	ros::ServiceServer executeCartesianTraj_server_;
	ros::ServiceServer executeCartesianTrajAsync_server_;

    // ROS ACTION SERVERS 
    actionlib::SimpleActionServer<fc_msgs::GoToPoseAction> go_to_pose_as_;
	actionlib::SimpleActionServer<fc_msgs::GoToPoseAction> go_to_pose_async_as_;
	actionlib::SimpleActionServer<fc_msgs::GoToJointsAction> go_to_joints_as_;
	actionlib::SimpleActionServer<fc_msgs::ExecuteCartesianTrajectoryAction> execute_cartesian_trajectory_as_;

    //ROS ACTION MSGS FEEDBACKS/RESULTS
	fc_msgs::GoToPoseFeedback go_to_pose_feedback_;
	fc_msgs::GoToPoseResult go_to_pose_result_;
	fc_msgs::GoToPoseFeedback go_to_pose_async_feedback_;
	fc_msgs::GoToPoseResult go_to_pose_async_result_;
	fc_msgs::GoToJointsFeedback go_to_joints_feedback_;
	fc_msgs::GoToJointsResult go_to_joints_result_;
	fc_msgs::ExecuteCartesianTrajectoryFeedback execute_cartesian_trajectory_feedback_;
	fc_msgs::ExecuteCartesianTrajectoryResult execute_cartesian_trajectory_result_;

    //ROS MSGS
	sensor_msgs::JointState last_joints_;

    //ROS TOPIC SUBSCRIBERS
	ros::Subscriber check_moving_sub_;
	ros::Subscriber robot_status_sub_;
	ros::Subscriber trajectory_status_sub_;
	ros::Subscriber joint_states_sub_;
	ros::Publisher tool0_pose_pub_;
	Eigen::MatrixXd robot_DH_;
	Eigen::MatrixXd robot_base_frame_;

    //ROS DATA MEMBERS
	tf::TransformListener listener_;

    //NON-ROS DATA MEMBERS
	moveit::planning_interface::MoveGroupInterface move_group_;
	double max_velocity_scaling_factor_;
	double max_acceleration_scaling_factor_;
	bool executing_trajectory_=false;
	bool is_moving_=false;
	actionlib_msgs::GoalStatus traj_status_;
	int max_retries_ = 10;
	int try_count_;
	Eigen::Matrix4d _FK(const VectorJd& q, const Eigen::MatrixXd& DH, const Eigen::Matrix4d& base_frame, const bool& joint_rad);
	Eigen::MatrixXd loadFromFile_(std::string file_path, std::string key);

	//PRIVATE METHODS
	
	void calculatePoseError_(const geometry_msgs::Pose &pose1, const geometry_msgs::Pose &pose2, double &position_error, double &orientation_error);
	bool planAndExecuteCartesianPath_(const std::vector<geometry_msgs::Pose> &waypoints, 
									double eef_step, double jump_threshold, 
									double max_velocity_scaling_factor, double max_acceleration_scaling_factor,
									double blend_radius, bool async, std::string planner_id = "LIN");

	bool planAndExecutePose_(const geometry_msgs::Pose &pose, 
							double max_velocity_scaling_factor, double max_acceleration_scaling_factor,
							std::string planner_id = "LIN",
							bool async = false);

public:
	/**
	* @brief Constructor for the FC_Interface class.
	*
	* Initializes the MoveIt! MoveGroup interface and sets up all relevant ROS interfaces
	* including action servers, service servers, and topic subscribers for controlling
	* a robotic manipulator.
	*
	* @param group_name Name of the MoveIt! planning group to be used.
	* @param hn_ Reference to the ROS node handle for setting up communication.
	*
	* The constructor performs the following:
	* - Initializes MoveGroup with the given planning group.
	* - Sets up action servers for pose, joint, and Cartesian trajectory goals.
	* - Sets default velocity and acceleration scaling factors.
	* - Starts action servers.
	* - Subscribes to joint state and trajectory status topics for feedback monitoring.
	* - Advertises multiple ROS service servers for motion control.
	*
	* @note The MoveGroupInterface's planning frame, pose reference frame, and end-effector
	*       link are logged for debugging purposes.
	*/
	FC_Interface(std::string group_name, ros::NodeHandle &nh_);
	
	/**
	* @brief Callback function for the 'fc_get_pose' ROS service.
	*
	* This service retrieves the current end-effector pose of the manipulator in the MoveIt! planning scene.
	* It responds with the pose expressed in the reference frame used by the MoveGroup interface.
	*
	* @param req Request message (unused; included for compatibility with service definition).
	* @param res Response message that contains the current pose (geometry_msgs::Pose) of the manipulator.
	* @return True if the pose was successfully retrieved and assigned to the response.
	*
	* @note An AsyncSpinner is used here to ensure proper ROS callback handling during blocking operations.
	*/
	bool getPose(fc_msgs::GetPose::Request &req, fc_msgs::GetPose::Response &res);

	/**
	* @brief Callback function for the 'fc_set_pose' ROS service.
	*
	* This service receives a target end-effector pose and attempts to move the robot to that pose using MoveIt!.
	* The pose can be executed with optional velocity and acceleration scaling, and a specified trajectory type.
	*
	* @param req Service request containing:
	*  - req.pose: Target pose to move the end-effector to (geometry_msgs::Pose).
	*  - OPTIONAL req.max_velocity_scaling_factor: Optional scaling for velocity (range: 0.0–1.0).
	*  - OPTIONAL req.max_acceleration_scaling_factor: Optional scaling for acceleration (range: 0.0–1.0).
	*  - OPTIONAL req.traj_type: Type of trajectory to execute (e.g., Cartesian or joint-space).
	*
	* @param res Service response containing:
	*  - res.pose: Actual end-effector pose after execution.
	*
	* @return true if the motion was successfully planned and executed, false otherwise.
	*
	* @note This function uses an AsyncSpinner to ensure ROS callbacks remain active during blocking operations.
 	*/
	bool setPose(fc_msgs::SetPose::Request &req, fc_msgs::SetPose::Response &res);
	
	/**
	* @brief Callback function for the 'fc_set_joints' ROS service.
	*
	* This service sets a target joint configuration for the manipulator and attempts to move
	* to it using the specified planner (in this case, BiTRRT). It also includes retry logic
	* in case the motion times out.
	*
	* @param req Service request containing:
	*  - req.state: The desired joint positions (sensor_msgs::JointState).
	*
	* @param res Service response containing:
	*  - res.state: The actual joint positions after motion execution.
	*
	* @return true if the motion was successfully planned and executed, false otherwise.
	*
	* @note The planner used is explicitly set to "BiTRRT". The motion is retried up to
	*       `max_retries` times if the initial execution times out. The joint names and
	*       current joint values are stored in `last_joints_` for future reference.
	*/
	bool setJoints(fc_msgs::SetJoints::Request &req, fc_msgs::SetJoints::Response &res);
	
	/**
	* @brief Callback function for the 'fc_execute_trajectory' ROS service.
	*
	* Executes a precomputed trajectory provided in the request using MoveIt!'s MoveGroup interface.
	* This is useful for executing trajectories planned and returned earlier by a planner.
	*
	* @param req Service request containing:
	*  - req.trajectory: A full robot trajectory (moveit_msgs::RobotTrajectory) to execute.
	*
	* @param res Service response containing:
	*  - res.error_code: The result of the execution (moveit_msgs::MoveItErrorCodes).
	*
	* @return true if the request was handled (even if execution failed); false if the service itself failed.
	*
	* @note The following steps are performed:
	* - Stops any ongoing execution with `move_group_.stop()`.
	* - Loads the provided trajectory into a Plan object.
	* - Executes the trajectory using `move_group_.execute()`.
	* - The error code is returned in the response regardless of success or failure.
	*
	* @warning This function returns `true` even if execution fails, to indicate the service completed.
	*          You must check `res.error_code` to determine if the trajectory was successfully executed.
	*/
	bool executeTrajectory(moveit_msgs::ExecuteKnownTrajectory::Request &req, moveit_msgs::ExecuteKnownTrajectory::Response &res);

	/**
	* @brief Callback function for the 'fc_stop_trajectory' ROS service.
	*
	* This service stops any currently executing trajectory using MoveIt!'s MoveGroup interface.
	* It is useful for emergency stops or manual interruption of motion execution.
	*
	* @param req Empty standard Trigger service request.
	* @param res Standard Trigger service response containing:
	*  - res.success: Always set to true if `move_group_.stop()` was called.
	*  - res.message: Informative string indicating the trajectory was stopped.
	*
	* @return true indicating the service call was processed successfully.
	*
	* @note Uses an AsyncSpinner to ensure the ROS callback queue remains active during execution.
	*/
	bool stopTrajectory(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res);

	/**
	 * @brief Callback function for the 'fc_execute_cartesian_trajectory' ROS service.
	 *
	 * Executes a Cartesian path through a series of waypoints using MoveIt!'s planning interface.
	 * The path is interpolated based on the specified end-effector step size and jump threshold.
	 *
	 * @param req Service request containing:
	 *  - req.poses: Sequence of geometry_msgs::Pose waypoints to follow.
	 *  - req.eef_step: Resolution of interpolation in meters along the Cartesian path.
	 *  - req.jump_threshold: Jump detection threshold for joint space discontinuities.
	 *  - req.max_velocity_scaling_factor: Optional scaling factor for velocity (range 0.0–1.0).
	 *  - req.max_acceleration_scaling_factor: Optional scaling factor for acceleration (range 0.0–1.0).
	 *  - req.blend_radius: Optional radius for blending waypoints (if supported).
	 *
	 * @param res Service response containing:
	 *  - res.pose: The final end-effector pose after executing the Cartesian trajectory.
	 *
	 * @return true if the Cartesian path was successfully planned and executed; false otherwise.
	 *
	 *
	 * @warning If planning fails due to unreachable waypoints or joint limits, the function returns false.
	*/
	bool executeCartesianTrajectory(fc_msgs::ExecuteCartesianTrajectory::Request &req, fc_msgs::ExecuteCartesianTrajectory::Response &res);
	
	/**
	 * @brief Callback function for the 'fc_execute_cartesian_trajectory_async' ROS service.
	 *
	 * Executes a Cartesian path asynchronously through a series of end-effector waypoints using MoveIt!.
	 * This is similar to `executeCartesianTrajectory`, but it does **not block** until the full trajectory is complete.
	 *
	 * @param req Service request containing:
	 *  - req.poses: A sequence of Cartesian waypoints (geometry_msgs::Pose) to follow.
	 *  - req.eef_step: Interpolation resolution in meters along the Cartesian path.
	 *  - req.jump_threshold: Threshold for detecting large joint-space jumps.
	 *  - req.max_velocity_scaling_factor: Optional scaling factor for motion velocity (range 0.0–1.0).
	 *  - req.max_acceleration_scaling_factor: Optional scaling factor for motion acceleration (range 0.0–1.0).
	 *  - req.blend_radius: Radius for blending between waypoints (if supported).
	 *
	 * @param res Service response containing:
	 *  - res.pose: The actual end-effector pose after initiating trajectory execution.
	 *
	 * @return true if the Cartesian trajectory was successfully planned and execution was started; false otherwise.
	 *
	 */
	bool executeCartesianTrajectoryAsync(fc_msgs::ExecuteCartesianTrajectory::Request &req, fc_msgs::ExecuteCartesianTrajectory::Response &res);
	
	/**
	 * @brief Callback function for the 'fc_go_to_joints' action server.
	 *
	 * This action moves the manipulator to a target joint configuration provided in the goal message.
	 * It uses MoveIt!'s MoveGroup interface to plan and execute the motion using the BiTRRT planner.
	 * Feedback is published during execution, and the final joint state is returned in the result.
	 *
	 * @param goal A shared pointer to the action goal containing:
	 *  - goal->state: Target joint configuration (sensor_msgs::JointState).
	 *  - goal->max_velocity_scaling_factor: Scaling factor for motion velocity (0.0 to 1.0).
	 *  - goal->max_acceleration_scaling_factor: Scaling factor for motion acceleration (0.0 to 1.0).
	 *
	 * @return None. The result is sent back using the action server handle:
	 * - If the move succeeds, `setSucceeded()` is called.
	 * - If the move fails, `setAborted()` is called with appropriate feedback.
	 *
	 * @warning If motion continuously times out or fails, the action is aborted with a failure result.
	 * @warning No upper bound is placed on retry attempts (could be improved for fault tolerance).
	 */
	void goToJointsCallback(const fc_msgs::GoToJointsGoalConstPtr &goal);

	/**
	 * @brief Callback function for the 'fc_go_to_pose' action server.
	 *
	 * This action moves the robot's end-effector to a specified pose using MoveIt!.
	 * The pose execution can be configured with velocity and acceleration scaling factors,
	 * and a selectable trajectory type (e.g., joint-space or Cartesian).
	 *
	 * @param goal A shared pointer to the action goal containing:
	 *  - goal->pose: The target end-effector pose (geometry_msgs::Pose).
	 *  - goal->max_velocity_scaling_factor: Scaling factor for velocity (range: 0.0–1.0).
	 *  - goal->max_acceleration_scaling_factor: Scaling factor for acceleration (range: 0.0–1.0).
	 *  - goal->traj_type: Integer or enum indicating the desired trajectory type.
	 *
	 * @return None. The result is communicated via `setSucceeded()` or `setAborted()` on the action server.
	 *
	 */
	void goToPoseCallback(const fc_msgs::GoToPoseGoalConstPtr &goal);

	/**
	 * @brief Callback function for the 'fc_go_to_pose_async' action server.
	 *
	 * This action moves the robot's end-effector to a specified pose using MoveIt! in **asynchronous** (non-blocking) mode.
	 * The target pose is provided in the goal, along with optional velocity and acceleration scaling factors,
	 * and a selectable trajectory type (e.g., joint-space or Cartesian).
	 *
	 * @param goal A shared pointer to the action goal containing:
	 *  - goal->pose: The target end-effector pose (geometry_msgs::Pose).
	 *  - goal->max_velocity_scaling_factor: Scaling factor for velocity (range: 0.0–1.0).
	 *  - goal->max_acceleration_scaling_factor: Scaling factor for acceleration (range: 0.0–1.0).
	 *  - goal->traj_type: Integer or enum indicating the desired trajectory type.
	 *
	 * @return None. The result is communicated using `setSucceeded()` or `setAborted()` via the action server handle.
	 */
	void goToPoseAsyncCallback(const fc_msgs::GoToPoseGoalConstPtr &goal);	

	/**
	 * @brief Callback function for the 'fc_execute_cartesian_trajectory' action server.
	 *
	 * This action executes a Cartesian trajectory defined by a sequence of waypoints.
	 * It provides real-time feedback on the progress of waypoint tracking, including
	 * percentage completion and currently reached waypoint ID.
	 *
	 * @param goal A shared pointer to the action goal containing:
	 *  - goal->waypoints: A list of geometry_msgs::Pose targets for the end-effector to follow.
	 *  - goal->eef_step: Resolution of interpolation in meters between waypoints.
	 *  - goal->jump_threshold: Threshold for joint-space discontinuity detection.
	 *  - goal->max_velocity_scaling_factor: Optional velocity scaling factor (range: 0.0–1.0).
	 *  - goal->max_acceleration_scaling_factor: Optional acceleration scaling factor (range: 0.0–1.0).
	 *  - goal->blend_radius: Radius to apply for smoothing transitions between waypoints (if supported).
	 *
	 * @note
	 * - Internally uses `planAndExecuteCartesianPath_()` with `async=true`.
	 * - Publishes `feedback` including:
	 *   - `feedback`: A string status message.
	 *   - `wp_id`: Index of the last reached waypoint.
	 *   - `wp_pct`: Percentage of waypoints reached.
	 * - Uses `calculatePoseError_()` to determine when the robot has reached the next waypoint.
	 * - Automatically handles preemption requests and aborts gracefully if preempted or ROS is shutting down.
	 *
	 * @return None. The result is returned via the action server using `setSucceeded()` or `setPreempted()`.
	 */
	void executeCartesianTrajectoryCallback(const fc_msgs::ExecuteCartesianTrajectoryGoalConstPtr & goal);
	
	/**
	 * @brief Callback function for the '/joint_states' topic.
	 *
	 * This function processes incoming joint state messages, computes the corresponding end-effector
	 * pose using forward kinematics, and publishes the result as a `geometry_msgs::PoseStamped` message.
	 *
	 * @param msg A shared pointer to the incoming sensor_msgs::JointState message containing joint positions.
	 *
	 * @details
	 * - Converts the joint positions into an Eigen vector `q`.
	 * - Calls `_FK()` to compute the transformation matrix from the base frame to the tool0 (end-effector).
	 * - Extracts the translation and rotation (as quaternion) from the 4x4 pose matrix.
	 * - Publishes the computed pose on `tool0_pose_pub_` as a `PoseStamped` message.
	 *
	 * @note
	 * - The base frame is assumed to be `"base"` (set as `frame_id`).
	 * - Orientation is extracted from the 3x3 rotation block of the transformation matrix.
	 * - `_FK()` is expected to implement the forward kinematics logic using the provided DH parameters.
	 */
	void jointStatesCallback(const sensor_msgs::JointState::ConstPtr& msg);

	/** Helper Functions */
	void checkMoving(const sensor_msgs::JointState::ConstPtr &msg);
	void checkTrajStatus(const actionlib_msgs::GoalStatusArray::ConstPtr &msg);

}; 

#endif