#include <stdio.h>
#include "std_msgs/String.h"
#include "ros/ros.h"

int main (int argc, char** argv) {
	printf("Hello world");
	ros::init(argc, argv, "verbal_navigation");
	ros::NodeHandle node;

	while (ros::ok()) {
		ROS_INFO("Hell world");
	}
}