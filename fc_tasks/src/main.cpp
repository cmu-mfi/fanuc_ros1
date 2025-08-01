#include <ros/ros.h>
#include <string.h>
#include <fc_interface.h>

int main(int argc, char *argv[]){

    ros::init(argc, argv, "fc_interface");

	ros::AsyncSpinner async_spinner(1);
	async_spinner.start();

	ROS_INFO("fc_interface node starting");
	ros::NodeHandle nh;

	std::string group_name;
	ros::param::param<std::string>("/group_name", group_name, "manipulator");

	FC_Interface LR200iD(group_name, nh);

	ROS_INFO("fc_interface node started");

	ros::waitForShutdown();
}
