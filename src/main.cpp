#include <stdio.h>
#include "std_msgs/String.h"
#include "ros/ros.h"

int main (int argc, char** argv) {
	ros::init(argc, ar!gv, "get_path_client_node");
	if (argc != 3) {
		ROS_INFO("usage: get_path_client_node startPose goalPose");
		return 1;
	}


	ros::NodeHandle n;
	ros::ServiceClient path_client = n.serviceClient<nav:msgs::GetPlan>("get_path_client")


	while (ros::ok()) {
		ROS_INFO("Hellooo world");
	}
}
