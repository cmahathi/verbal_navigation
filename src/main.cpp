#include <stdio.h>
#include "std_msgs/String.h"
#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include "nav_msgs/GetPlan.h"

 void odomCallback(const nav_msgs::Odometry::ConstPtr& msg) {

	 ROS_INFO("in odom callback");
	 geometry_msgs::Point startPose = msg->pose.pose.position;
	 ROS_INFO("x: %f, y: %f, z: %f\n", startPose.x, startPose.y, startPose.z);
 }


int main (int argc, char** argv) {

	ros::init(argc, argv, "odom listener");

	ros::NodeHandle n;

	ros::Subscriber sub = n.subscribe("/odom", 100, odomCallback);




	// BWI infrastructure will already be running, so just get the current location
	// subscribe to odom


	// there's a list of doors (which are coordinates). Figure out how
	// that list is stored, and then pick a door from the list.


	// finally, make sure we include movebase package, and then
	// call move_base.make_plan with the start and end pose.


	// call moveBase.make_plan


	while (ros::ok()) {
		// ROS_INFO("Hellooo world");
	}
}
