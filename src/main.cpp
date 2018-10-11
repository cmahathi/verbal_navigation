#include <stdio.h>
#include "std_msgs/String.h"
#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include "nav_msgs/GetPlan.h"
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>


class FuturePoseStamped {
	geometry_msgs::PoseStamped myPose;
	bool available;

public:
	FuturePoseStamped() : myPose(), available(false) { }

	 void setFromPoseWithCovarianceStamped(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg) {

		 ROS_INFO("in setFromPoseWithCovarianceStamped");
		 geometry_msgs::Point startPose = msg->pose.pose.position;
		 ROS_INFO("x: %f, y: %f, z: %f\n", startPose.x, startPose.y, startPose.z);

		 myPose.pose = msg->pose.pose;
		 myPose.header.stamp = ros::Time::now();
		 myPose.header.frame_id = "/level_mux_map";
		 available = true;
	 }

	 void setFromPoseStamped(const geometry_msgs::PoseStamped::ConstPtr& msg) {

		 ROS_INFO("in setFromPoseStamped");
		 geometry_msgs::Point startPose = msg->pose.position;
		 ROS_INFO("x: %f, y: %f, z: %f\n", startPose.x, startPose.y, startPose.z);

		 myPose.pose = msg->pose;
		 myPose.header.stamp = ros::Time::now();
		 myPose.header.frame_id = "/level_mux_map";
		 available = true;
	 }

	 bool isAvailable() {
	 	return available;
	 }
};


int main (int argc, char** argv) {

	FuturePoseStamped initialPose;
	FuturePoseStamped goalPose;

	ros::init(argc, argv, "main_node");

	ros::NodeHandle n;
	ros::Subscriber sub = n.subscribe("/initialpose", 100, &FuturePoseStamped::setFromPoseWithCovarianceStamped, &initialPose);
	ros::Subscriber sub1 = n.subscribe("/move_base_interruptable_simple/goal", 100, &FuturePoseStamped::setFromPoseStamped, &goalPose);

	while(!(initialPose.isAvailable() && goalPose.isAvailable())) ros::spinOnce();
	ROS_INFO("HAVE START AND GOAL");
	ros::ServiceClient path_client = n.serviceClient <nav_msgs::GetPlan> ("move_base/NavfnROS/make_plan");
	path_client.waitForExistence();
	ROS_INFO("Path client found");

	nav_msgs::GetPlan srv;
	// geometry_msgs::PoseStamped &start = srv.request.start;
	// geometry_msgs::PoseStamped &goal = srv.request.goal;

	// start.header.frame_id = goal.header.frame_id = "/level_mux_map";

	// start.header.stamp = goal.header.stamp = ros::Time::now();




	// tf::TransformListener listener;
	// geometry_msgs::PointStamped startPoint;
	// geometry_msgs::PointStamped startPointT;
	// startPoint.point.x = 0;
	// startPoint.point.y = 0;
	// startPoint.point.z = 0;
	// startPoint.header.frame_id = "/base_link";

	// geometry_msgs::PointStamped endPoint;
	// geometry_msgs::PointStamped endPointT;
	// endPoint.point.x = 5;
	// endPoint.point.y = 0;
	// endPoint.point.z = 0;
	// endPoint.header.frame_id = "/base_link";

	// endPoint.header.stamp = startPoint.header.stamp = ros::Time::now();
	// listener.waitForTransform("/base_link", "level_mux_map", ros::Time(0), ros::Duration(4));
 //    listener.transformPoint("level_mux_map", startPoint, startPointT);
 //    listener.transformPoint("level_mux_map", endPoint, endPointT);

	// start.pose.position = startPointT.point;
	// goal.pose.position = endPointT.point;

	// start.pose.orientation.x = 0;
	// start.pose.orientation.y = 0;
	// start.pose.orientation.z = 0;
	// start.pose.orientation.w = 1;

	// goal.pose.orientation.x = 0;
	// goal.pose.orientation.y = 0;
	// goal.pose.orientation.z = 0;
	// goal.pose.orientation.w = 1;

	// srv.request.tolerance = -1.0f;

	// path_client.call(srv);
	// ROS_INFO("Response received, size %d", srv.response.plan.poses.size());




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
