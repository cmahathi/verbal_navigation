#include <stdio.h>
#include <string.h>
#include "std_msgs/String.h"
#include "ros/ros.h"
#include "nav_msgs/GetPlan.h"
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <bwi_planning_common/structures.h>
#include <bwi_logical_translator/bwi_logical_translator.h>
#include <bwi_mapper/structures/point.h>
#include <vector>
#include "verbal_navigation/FuturePoseStamped.h"
#include "verbal_navigation/RegionPath.h"

const std::string projectDir = "/home/fri/TGI_FRIdays_ws/";

std::string getRegion(bwi_logical_translator::BwiLogicalTranslator& translator, geometry_msgs::PoseStamped currentLocation) {
	float robot_x = currentLocation.pose.position.x;
	float robot_y = currentLocation.pose.position.y;

	bwi_mapper::Point2f mapPoint(robot_x, robot_y);
	return translator.getLocationString(translator.getLocationIdx(mapPoint));
	// return translator.getRegionString(translator.getRegionIdx(mapPoint));
}

int main (int argc, char** argv) {

	FuturePoseStamped initialPose;
	FuturePoseStamped goalPose;

	ros::init(argc, argv, "main_node");

	ros::NodeHandle n;
	ros::Subscriber sub = n.subscribe("/initialpose", 100, &FuturePoseStamped::setFromPoseWithCovarianceStamped, &initialPose);
	ros::Subscriber sub1 = n.subscribe("/move_base_interruptable_simple/goal", 100, &FuturePoseStamped::setFromPoseStamped, &goalPose);

	ros::param::set("~map_file", projectDir +  "src/verbal_navigation/src/3ne/3ne.yaml");
	ros::param::set("~data_directory", projectDir + "src/verbal_navigation/src/3ne");
	bwi_logical_translator::BwiLogicalTranslator translator;
	translator.initialize();


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
	std::vector<geometry_msgs::PoseStamped> pose_list = srv.response.plan.poses;

	RegionPath regionPath(translator, pose_list);
	// std::vector<std::string> region_list;
	// region_list.push_back(getRegion(translator, pose_list.front()));
	//
	// for(size_t i = 0; i < pose_list.size(); ++i) {
	// 	auto region = getRegion(translator, pose_list.at(i));
	// 	if(region_list.back().compare(region) != 0) {
	// 		region_list.push_back(region);
	// 		ROS_INFO("Added region: %s\n", region.c_str());
	// 	}
	// }
	// TODO: pass srv.response into new function (compartmentalize!!)

	// there's a list of doors (which are coordinates). Figure out how
	// that list is stored, and then pick a door from the list.





	while (ros::ok()) {
		// ROS_INFO("Hellooo world");
		ros::spinOnce();
	}
}
