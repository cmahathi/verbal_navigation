#include <verbal_navigation/FuturePoseStamped.h>


FuturePoseStamped::FuturePoseStamped() : myPose(), available(false) { }

void FuturePoseStamped::setFromPoseWithCovarianceStamped(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg) {
	ROS_INFO("in setFromPoseWithCovarianceStamped");
	geometry_msgs::Point startPose = msg->pose.pose.position;
	//geometry_msgs::orientation orient = msg->pose.pose.orientation;
	ROS_INFO("x: %f, y: %f, z: %f\n", startPose.x, startPose.y, startPose.z);
	ROS_INFO("Orientation: %f %f %f %f\n",  msg->pose.pose.orientation.x,  msg->pose.pose.orientation.y,  msg->pose.pose.orientation.z,  msg->pose.pose.orientation.w);

	myPose.pose = msg->pose.pose;
	myPose.header.stamp = ros::Time::now();
	myPose.header.frame_id = "/level_mux_map";
	available = true;
}

void FuturePoseStamped::setFromPoseStamped(const geometry_msgs::PoseStamped::ConstPtr& msg) {
	ROS_INFO("in setFromPoseStamped");
	geometry_msgs::Point startPose = msg->pose.position;
	//geometry_msgs::orientation orient = msg->pose.pose.orientation;
	ROS_INFO("x: %f, y: %f, z: %f\n", startPose.x, startPose.y, startPose.z);
	ROS_INFO("Orientation: %f %f %f %f\n",  msg->pose.orientation.x,  msg->pose.orientation.y,  msg->pose.orientation.z,  msg->pose.orientation.w);

	myPose.pose = msg->pose;
	myPose.header.stamp = ros::Time::now();
	myPose.header.frame_id = "/level_mux_map";
	available = true;
}

bool FuturePoseStamped::isAvailable() {
	return available;
}

geometry_msgs::PoseStamped FuturePoseStamped::getPose() {
	return myPose;
}

void FuturePoseStamped::reset() {
	available = false;
	myPose = geometry_msgs::PoseStamped();
}
