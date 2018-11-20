#include <stdio.h>
#include <string.h>
#include "std_msgs/String.h"
#include "ros/ros.h"
#include "nav_msgs/GetPlan.h"
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
#include "verbal_navigation/Preposition.h"
#include "verbal_navigation/VerbPhrase.h"
#include "verbal_navigation/MapItem.h"

// NOTE: this path might be wrong, check before running on different machines
const std::string projectDir = boost::filesystem::current_path().string();

void sleepok(int t, ros::NodeHandle &nh) {
	if (nh.ok()) {
		sleep(t);
	}
}


int main (int argc, char** argv) {
	ROS_INFO("Welcome to FRI_SPEAK");


	ros::init(argc, argv, "FRI_SPEAK");
	ros::NodeHandle nh("~");

	// parameter destination d3_432 should be specified as ROS param

	// from the ros tutorials
	std::string destinationName = "";
	if (nh.getParam("dest", destinationName))
	{
  	ROS_INFO("Got param: %s", destinationName.c_str());
	}
	else
	{
  	ROS_ERROR("Failed to get param 'dest'");
	}



	FuturePoseStamped initialPose;
	// FuturePoseStamped goalPose;

	// geometry_msgs::PoseStamped initialPose;
	// geometry_msgs::PoseStamped goalPose;

	//for now, hard code the start and end poses
	// initialPose.header.stamp = ros::Time::now();
	// initialPose.header.frame_id = "/level_mux_map";
	// initialPose.pose.position.x = 14.6793708801;
	// initialPose.pose.position.y = 110.153694153;
	// initialPose.pose.position.z = 0.0;
	// initialPose.pose.orientation.x = 0.0;
	// initialPose.pose.orientation.y = 0.0;
	// initialPose.pose.orientation.z = -0.04113175032;
	// initialPose.pose.orientation.w = 0.999153731473;
	//
	// goalPose.header.stamp = ros::Time::now();
	// goalPose.header.frame_id = "/level_mux_map";
	// goalPose.pose.position.x = 47.8235244751;
	// goalPose.pose.position.y = 108.96232605;
	// goalPose.pose.position.z = 0.0;
	// goalPose.pose.orientation.x = 0.0;
	// goalPose.pose.orientation.y = 0.0;
	// goalPose.pose.orientation.z = -0.65703277302;
	// goalPose.pose.orientation.w = 0.753862013354;

	// get door location from user-specified door name



	//subscribe to topics which provide start and dest poses
	ros::Subscriber sub = nh.subscribe("/initialpose", 100, &FuturePoseStamped::setFromPoseWithCovarianceStamped, &initialPose);
	// ros::Subscriber sub1 = nh.subscribe("/move_base_interruptable_simple/goal", 100, &FuturePoseStamped::setFromPoseStamped, &goalPose);

	ros::param::set("~map_file", projectDir +  "/src/3ne/3ne.yaml");
	ros::param::set("~data_directory", projectDir + "/src/3ne");

	bwi_logical_translator::BwiLogicalTranslator translator;
	translator.initialize();

	// call service to generate path from start to dest
	// ros::ServiceClient path_client = n.serviceClient <nav_msgs::GetPlan> ("/move_base/NavfnROS/make_plan");
	ros::ServiceClient path_client = nh.serviceClient <nav_msgs::GetPlan> ("/move_base/make_plan");
	path_client.waitForExistence();
	ROS_INFO("Path service found!");

	while (ros::ok()) {
	// wait until start and dest poses exist
		while(!(initialPose.isAvailable() /* && goalPose.isAvailable() */) && ros::ok()) ros::spinOnce();

		ROS_INFO("Start and dest poses received! Generating path...");

		nav_msgs::GetPlan srv;
		srv.request.start = initialPose.getPose();
		geometry_msgs::PoseStamped goalPose;
		// Need to convert this point2f (pixel coords on map) to a poseStamped for our goal pose
		auto goalPoints = translator.getDoor(destinationName).door_center;
		goalPose.pose.position.x = goalPoints.x;
		goalPose.pose.position.y = goalPoints.y;
		goalPose.header.stamp = ros::Time::now();
		goalPose.header.frame_id = "/level_mux_map";

		srv.request.goal = goalPose;

		// srv.request.start = initialPose;
		// srv.request.goal = goalPose;

		srv.request.tolerance = -1.0f;

		// call service to generate plan, which returns a list of PoseStamped
		path_client.call(srv);

		ROS_INFO("Path received! Size: %d", srv.response.plan.poses.size());
		std::vector<geometry_msgs::PoseStamped> pose_list = srv.response.plan.poses;


		// do the heavy lifting in this class
		MapInfo mapInfo(translator, pose_list, destinationName);
		std::string finalDirections = mapInfo.generateDirections();
		ROS_INFO("***");
		ROS_INFO("FINAL DIRECTIONS: %s", finalDirections.c_str());
		ROS_INFO("***");

		// make a sound_play object, which will speak the final directions
		sound_play::SoundClient sc;
		sleepok(5, nh);
		// NOTE: MAKE SURE TO RUN THE sound_play node using
		// "rosrun sound_play soundplay_node.py" before sending a sound
		sc.say(finalDirections);

		// initialPose = goalPose;
		// goalPose.reset();
	}
}
