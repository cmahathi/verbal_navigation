#include <stdio.h>
#include "std_msgs/String.h"
#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include "nav_msgs/GetPlan.h"
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <bwi_planning_common/structures.h>
#include <bwi_logical_translator/bwi_logical_translator.h>



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

	 geometry_msgs::PoseStamped getPose() {
	 	return myPose;
	 }
};

void getRegion(FuturePoseStamped current_location) {
	std::string locationFileName = "/home/fri/catkin_ws/src/verbal_navigation/src/3ne/locations.yaml";
	std::vector<std::string> locations;
	std::vector<int32_t> location_map;

	bwi_planning_common::readLocationFile(locationFileName, locations, location_map);

	ROS_INFO("Size of locations: %d, and size of map: %d\n", locations.size(), location_map.size());   	

	tf::TransformListener tf;
	geometry_msgs::PoseStamped out;
	tf.transformPose("level_mux_map", current_location.getPose(), out);
	float robot_x = out.pose.position.x;
	float robot_y = out.pose.position.y;
	
	//Convert to cv::point2f using bwi_mapper::Point2f(robot_x, robot_y)
	//Call bwi_logical_translator::getLocationIdx(cv::Point2f&);
}

int main (int argc, char** argv) {

	
	
	FuturePoseStamped initialPose;
	FuturePoseStamped goalPose;

	ros::init(argc, argv, "main_node");

	ros::NodeHandle n;
	ros::Subscriber sub = n.subscribe("/initialpose", 100, &FuturePoseStamped::setFromPoseWithCovarianceStamped, &initialPose);
	ros::Subscriber sub1 = n.subscribe("/move_base_interruptable_simple/goal", 100, &FuturePoseStamped::setFromPoseStamped, &goalPose);


	
	while(!(initialPose.isAvailable() && goalPose.isAvailable()) && ros::ok()) ros::spinOnce();
	ROS_INFO("HAVE START AND GOAL");
	ros::ServiceClient path_client = n.serviceClient <nav_msgs::GetPlan> ("move_base/NavfnROS/make_plan");
	path_client.waitForExistence();
	ROS_INFO("Path client found");

	nav_msgs::GetPlan srv;
	srv.request.start = initialPose.getPose();
	srv.request.goal = goalPose.getPose();

	srv.request.tolerance = -1.0f;

	path_client.call(srv);
	ROS_INFO("Response received, size %d", srv.response.plan.poses.size());

	// TODO: pass srv.response into new function (compartmentalize!!)



	// there's a list of doors (which are coordinates). Figure out how
	// that list is stored, and then pick a door from the list.





	while (ros::ok()) {
		// ROS_INFO("Hellooo world");
		ros::spinOnce();
	}
}
