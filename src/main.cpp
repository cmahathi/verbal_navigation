#include <stdio.h>
#include "std_msgs/String.h"
#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include "nav_msgs/GetPlan.h"
#include <geometry_msgs/PoseStamped.h>

 void odomCallback(const nav_msgs::Odometry::ConstPtr& msg) {

	 ROS_INFO("in odom callback");
	 geometry_msgs::Point startPose = msg->pose.pose.position;
	 ROS_INFO("x: %f, y: %f, z: %f\n", startPose.x, startPose.y, startPose.z);
 }


int main (int argc, char** argv) {

	ros::init(argc, argv, "odom_listener");

	ros::NodeHandle n;

	ros::Subscriber sub = n.subscribe("/odom", 100, odomCallback);

	ros::ServiceClient path_client = n.serviceClient <nav_msgs::GetPlan> ("move_base/NavfnROS/make_plan");
	path_client.waitForExistence();
	ROS_INFO("Path client found");

	nav_msgs::GetPlan srv;
	geometry_msgs::PoseStamped &start = srv.request.start;
	geometry_msgs::PoseStamped &goal = srv.request.goal;

	start.header.frame_id = "base_link";
	goal.header.frame_id = "odom";

	start.header.stamp = goal.header.stamp = ros::Time::now();

	start.pose.position.x = 0;
	start.pose.position.y = 0;
	start.pose.position.z = 0;

	start.pose.orientation.x = 0;
	start.pose.orientation.y = 0;
	start.pose.orientation.z = 0;
	start.pose.orientation.w = 1;

	goal.pose.position.x = 0;
	goal.pose.position.y = 0;
	goal.pose.position.z = 0;

	goal.pose.orientation.x = 0;
	goal.pose.orientation.y = 0;
	goal.pose.orientation.z = 0;
	goal.pose.orientation.w = 1;

	srv.request.tolerance = -1.0f;

	path_client.call(srv);
	ROS_INFO("Response received, size %d", srv.response.plan.poses.size());





	// BWI infrastructure will already be running, so just get the current location
	// subscribe to odom


	// there's a list of doors (which are coordinates). Figure out how
	// that list is stored, and then pick a door from the list.


	// finally, make sure we include movebase package, and then
	// call move_base.make_plan with the start and end pose.


	// call moveBase.make_plan


	while (ros::ok()) {
		// ROS_INFO("Hellooo world");
		ros::spinOnce();
	}
}
