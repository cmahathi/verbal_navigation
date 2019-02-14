#include <stdio.h>
#include <string.h>
#include "std_msgs/String.h"
#include "ros/ros.h"
#include "ros/package.h"
#include "nav_msgs/GetPlan.h"
#include "multi_level_map_msgs/ChangeCurrentLevel.h"
#include <multi_level_map_utils/utils.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <bwi_planning_common/structures.h>
#include <bwi_logical_translator/bwi_logical_translator.h>
#include <bwi_mapper/structures/point.h>
#include <vector>
#include <boost/filesystem.hpp>

#include <sound_play/sound_play.h>
#include <unistd.h>

#include "verbal_navigation/FuturePoseStamped.h"
#include "verbal_navigation/MapInfo.h"
#include "verbal_navigation/Predicates.h"
#include "verbal_navigation/MapItem.h"
#include "verbal_navigation/bwi_directions_generator.h"

void sleepok(int t, ros::NodeHandle &nh) {
	if (nh.ok()) {
		sleep(t);
	}
}

geometry_msgs::PoseStamped::ConstPtr tryGetStartPose(std::map<std::string, geometry_msgs::Pose> landmarkNameToPositionMap, std::string startLandmark) {
	auto startPose = new geometry_msgs::PoseStamped;

	auto startPair = landmarkNameToPositionMap.find(startLandmark);
	if (startPair == landmarkNameToPositionMap.end()) {
		ROS_ERROR("start position landmark not found. Waiting for user-specified start pose from RViz.");
		//ros::Subscriber sub = nh.subscribe("/initialpose", 100, &FuturePoseStamped::setFromPoseWithCovarianceStamped, &initialPose);
	} else {
		startPose->pose = startPair->second;

		startPose->header.stamp = ros::Time::now();
		startPose->header.frame_id = "/level_mux_map";
	}
	
	return geometry_msgs::PoseStamped::ConstPtr(startPose);
}

void changeToFloor(ros::ServiceClient& change_floor_client, std::string floor_id) {
	ROS_INFO("Changing floor to %s", floor_id.c_str());
	multi_level_map_msgs::ChangeCurrentLevel changeLevel;
	
	changeLevel.request.new_level_id = floor_id;

	geometry_msgs::PoseWithCovarianceStamped origin_pose;
	origin_pose.header.frame_id = multi_level_map::frameIdFromLevelId(floor_id);
	ROS_INFO("Frame: %s", origin_pose.header.frame_id.c_str());
	origin_pose.pose.pose.orientation.w = 1;    // Makes the origin quaternion valid.
	origin_pose.pose.covariance[0] = 1.0;
	origin_pose.pose.covariance[7] = 1.0;
	origin_pose.pose.covariance[14] = 1.0;
	origin_pose.pose.covariance[21] = 1.0;
	origin_pose.pose.covariance[28] = 1.0;
	origin_pose.pose.covariance[35] = 1.0;

	changeLevel.request.initial_pose = origin_pose;

	change_floor_client.call(changeLevel);
}


int main (int argc, char** argv) {
	ROS_INFO("Welcome to the Verbal Navigation Porject");


	ros::init(argc, argv, "FRI_SPEAK");
	ros::NodeHandle nh("~");


	FuturePoseStamped initialPose;
	FuturePoseStamped goalPose;

	// geometry_msgs::PoseStamped initialPose;
	// geometry_msgs::PoseStamped goalPose;

	// get door location from user-specified door name

	ros::Subscriber sub1 = nh.subscribe("/move_base_interruptable_simple/goal", 100, &FuturePoseStamped::setFromPoseStamped, &goalPose);
	ros::Subscriber sub = nh.subscribe("/initialpose", 100, &FuturePoseStamped::setFromPoseWithCovarianceStamped, &initialPose);

	// SWAP FLOOR MAPS HERE - possibly automate later
	std::string projectDir = ros::package::getPath("verbal_navigation");
	boost::filesystem::path mapPath2 = projectDir + "/src/multimap/2/2.yaml";
	boost::filesystem::path dataPath2 = projectDir + "/src/multimap/2";
	boost::filesystem::path mapPath3 = projectDir + "/src/multimap/3ne/3ne.yaml";
	boost::filesystem::path dataPath3 = projectDir + "/src/multimap/3ne";

	std::vector<geometry_msgs::PoseStamped> pose_list;
	bwi_directions_generator::BwiDirectionsGenerator directionsGenerator;
	std::string destinationName = "";
	std::string finalDirections;

	// call service to generate path from start to dest
	// ros::ServiceClient path_client = n.serviceClient <nav_msgs::GetPlan> ("/move_base/NavfnROS/make_plan");
	ros::ServiceClient path_client = nh.serviceClient <nav_msgs::GetPlan> ("/move_base/NavfnROS/make_plan");
	
	path_client.waitForExistence();
	// ROS_INFO("Path service found!");

	// bwi_logical_translator::BwiLogicalTranslator translator3;
	// ros::param::set("~map_file", mapPath3.string());
	// ros::param::set("~data_directory", dataPath3.string());
	// translator3.initialize();

	// const auto& landmarkNameToPositionMap3 = translator3.getObjectApproachMap();
	// auto startPair3 = landmarkNameToPositionMap3.find("start");
	// auto destPair3 = landmarkNameToPositionMap3.find("dest");

	// auto startPose3 = new geometry_msgs::PoseStamped;
	// startPose3->pose = startPair3->second;

	// startPose3->header.stamp = ros::Time::now();
	// startPose3->header.frame_id = "/level_mux_map";

	// geometry_msgs::PoseStamped::ConstPtr start3(startPose3);
	// initialPose.setFromPoseStamped(start3);

	// auto endPose = new geometry_msgs::PoseStamped;
	// endPose->pose = destPair3->second;

	// endPose->header.stamp = ros::Time::now();
	// endPose->header.frame_id = "/level_mux_map";

	// geometry_msgs::PoseStamped::ConstPtr goal3(endPose);
	// goalPose.setFromPoseStamped(goal3);
	


	// nav_msgs::GetPlan srv3;

	// srv3.request.start = initialPose.getPose();
	// srv3.request.goal = goalPose.getPose();

	// srv3.request.tolerance = -1.0f;

	// // call service to generate plan, which returns a list of PoseStamped
	// path_client.call(srv3);

	// ROS_INFO("Path received! Size: %d", srv3.response.plan.poses.size());
	// pose_list = srv3.response.plan.poses;


	// // do the heavy lifting in this class
	// MapInfo mapInfo3 = directionsGenerator.GenerateDirectionsForPathOnMap(pose_list, mapPath3, destinationName, "3ne");
	// finalDirections = mapInfo3.generateDirections();
	// ROS_INFO("***");
	// ROS_INFO("FINAL DIRECTIONS: %s", finalDirections.c_str());
	// ROS_INFO("***");

	ros::ServiceClient change_level_client = nh.serviceClient <multi_level_map_msgs::ChangeCurrentLevel> ("/level_mux/change_current_level");

	change_level_client.waitForExistence();
	ROS_INFO("Able to change levels!");

	changeToFloor(change_level_client, "2ndFloor");
	sleep(1);
	// get the landmark "start"
	bwi_logical_translator::BwiLogicalTranslator translator2;
	ros::param::set("~map_file", mapPath2.string());
	ros::param::set("~data_directory", dataPath2.string());
	translator2.initialize();

	const auto& landmarkNameToPositionMap = translator2.getObjectApproachMap();
	auto startPose = tryGetStartPose(landmarkNameToPositionMap, "start");

	initialPose.setFromPoseStamped(startPose);

	// from the ros tutorials, get the destination door
	
	if (nh.getParam("dest", destinationName))
	{
		// ROS_INFO("reading user-specified dest paramter.");
		auto destination = new geometry_msgs::PoseStamped;

		auto goalPoints = translator2.getDoor(destinationName).approach_points[0];
		// ROS_INFO("FOUND DOOR");
		// Need to convert this point2f (pixel coords on map) to a poseStamped for our goal pose
		destination->pose.position.x = goalPoints.x;
		destination->pose.position.y = goalPoints.y;

		destination->header.stamp = ros::Time::now();
		destination->header.frame_id = "/level_mux_map";

		geometry_msgs::PoseStamped::ConstPtr goal(destination);
		goalPose.setFromPoseStamped(goal);
	}
	else
	{
		// user did not specify goal pose. Allow user to specify goal pose from RViz.
  		ROS_ERROR("Failed to get param 'dest'. Waiting for user to specify goal pose in RViz.");
		// ros::Subscriber sub1 = nh.subscribe("/move_base_interruptable_simple/goal", 100, &FuturePoseStamped::setFromPoseStamped, &goalPose);

	}


	

	//while (ros::ok()) {
	// wait until start and dest poses exist
	while(!(initialPose.isAvailable() && goalPose.isAvailable()) && ros::ok()) ros::spinOnce();

	// ROS_INFO("Start and dest poses received! Generating path...");

	nav_msgs::GetPlan srv;

	srv.request.start = initialPose.getPose();
	srv.request.goal = goalPose.getPose();

	srv.request.tolerance = -1.0f;

	// call service to generate plan, which returns a list of PoseStamped
	path_client.call(srv);

	//ROS_INFO("Path received! Size: %d", srv.response.plan.poses.size());
	pose_list = srv.response.plan.poses;


	// do the heavy lifting in this class
	MapInfo mapInfo = directionsGenerator.GenerateDirectionsForPathOnMap(pose_list, mapPath2, destinationName, "2");
	finalDirections = mapInfo.generateDirections();
	ROS_INFO("***");
	ROS_INFO("FINAL DIRECTIONS: %s", finalDirections.c_str());
	ROS_INFO("***");

	changeToFloor(change_level_client, "3rdFloor");
	sleep(1);

	
	bwi_logical_translator::BwiLogicalTranslator translator3;
	ros::param::set("~map_file", mapPath3.string());
	ros::param::set("~data_directory", dataPath3.string());
	translator3.initialize();

	const auto& landmarkNameToPositionMap3 = translator3.getObjectApproachMap();
	auto startPair3 = landmarkNameToPositionMap3.find("start");
	auto destPair3 = landmarkNameToPositionMap3.find("dest");

	auto startPose3 = new geometry_msgs::PoseStamped;
	startPose3->pose = startPair3->second;

	startPose3->header.stamp = ros::Time::now();
	startPose3->header.frame_id = "/level_mux_map";

	geometry_msgs::PoseStamped::ConstPtr start3(startPose3);
	initialPose.setFromPoseStamped(start3);

	auto endPose = new geometry_msgs::PoseStamped;
	endPose->pose = destPair3->second;

	endPose->header.stamp = ros::Time::now();
	endPose->header.frame_id = "/level_mux_map";

	geometry_msgs::PoseStamped::ConstPtr goal3(endPose);
	goalPose.setFromPoseStamped(goal3);
	


	nav_msgs::GetPlan srv3;

	srv3.request.start = initialPose.getPose();
	srv3.request.goal = goalPose.getPose();

	srv3.request.tolerance = -1.0f;

	// call service to generate plan, which returns a list of PoseStamped
	path_client.call(srv3);

	ROS_INFO("Path received! Size: %d", srv3.response.plan.poses.size());
	pose_list = srv3.response.plan.poses;


	// do the heavy lifting in this class
	MapInfo mapInfo3 = directionsGenerator.GenerateDirectionsForPathOnMap(pose_list, mapPath3, destinationName, "3ne");
	finalDirections = mapInfo3.generateDirections();
	ROS_INFO("***");
	ROS_INFO("FINAL DIRECTIONS: %s", finalDirections.c_str());
	ROS_INFO("***");

	std::vector<Region> regionPath;
	

		/*

		// make a sound_play object, which will speak the final directions
		sound_play::SoundClient sc;
		sleepok(5, nh);
		// NOTE: MAKE SURE TO RUN THE sound_play node using
		// "rosrun sound_play soundplay_node.py" before sending a sound
		sc.say(finalDirections);

		*/

	// update robot's position and set up for new goal pose.
	// initialPose = goalPose;
	// goalPose.reset();
// }


}

// These functions are needed in bwi_logical_translator.h
// inline bwi_planning_common::Door getDoor(std::string door_str) const {
// 	return doors_.at(getDoorIdx(door_str));
// }
//
// inline std::map<std::string, geometry_msgs::Pose> getObjectApproachMap() const {
// 	return object_approach_map_;
// }
