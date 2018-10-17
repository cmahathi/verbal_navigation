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
#include <verbal_navigation/FuturePoseStamped.h>
#include <vector>



std::string getRegion(bwi_logical_translator::BwiLogicalTranslator translator, geometry_msgs::PoseStamped currentLocation) {
	float robot_x = currentLocation.pose.position.x;
	float robot_y = currentLocation.pose.position.y;
	ROS_INFO("X: %f Y: %f\n", robot_x, robot_y); 
	
	bwi_mapper::Point2f mapPoint(robot_x, robot_y);
	int index = translator.getLocationIdx(mapPoint);
	ROS_INFO("IDX: %d\n", index);
	std::string region = (translator.getLocationString(index)).c_str();
	ROS_INFO("mapPoint created. Found region %s", region);	
	return region;
}

int main (int argc, char** argv) {

	FuturePoseStamped initialPose;
	FuturePoseStamped goalPose;

	ros::init(argc, argv, "main_node");

	ros::NodeHandle n;
	ros::Subscriber sub = n.subscribe("/initialpose", 100, &FuturePoseStamped::setFromPoseWithCovarianceStamped, &initialPose);
	ros::Subscriber sub1 = n.subscribe("/move_base_interruptable_simple/goal", 100, &FuturePoseStamped::setFromPoseStamped, &goalPose);

	ros::param::set("~map_file", "/home/users/fri/TGI_FRIdays_ws/src/verbal_navigation/src/3ne/3ne.yaml");
	ros::param::set("~data_directory", "/home/users/fri/TGI_FRIdays_ws/src/verbal_navigation/src/3ne");
	bwi_logical_translator::BwiLogicalTranslator translator;
	translator.initialize();


	while(!(initialPose.isAvailable() && goalPose.isAvailable()) && ros::ok()) ros::spinOnce();

 	ROS_INFO("%s\n", getRegion(translator, initialPose.getPose()));

	//getRegion(translator, initialPose);
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

	std::vector<std::string> region_list;

	for (int i = 0; i < pose_list.size(); i++) {
		//std::string region = getRegion(translator, pose_list.at(i));
		geometry_msgs::PoseStamped currentLocation = pose_list.at(i);
		float robot_x = currentLocation.pose.position.x;
		float robot_y = currentLocation.pose.position.y;
		ROS_INFO("X: %f Y: %f\n", robot_x, robot_y); 
		
		//bwi_mapper::Point2f mapPoint(robot_x, robot_y);
		std::string region = (translator.getLocationString(translator.getLocationIdx(bwi::Point2f(robot_x, robot_y))));
		ROS_INFO("mapPoint created. Found region %s", region);
			if (region.compare(region_list.back()) != 0)
				region_list.push_back(region); 
	}

	for (int j = 0; j < region_list.size(); j++) {
		ROS_INFO ("Region: %s\n", region_list.at(j));
	}


	// TODO: pass srv.response into new function (compartmentalize!!)



	// there's a list of doors (which are coordinates). Figure out how
	// that list is stored, and then pick a door from the list.





	while (ros::ok()) {
		// ROS_INFO("Hellooo world");
		ros::spinOnce();
	}
}
