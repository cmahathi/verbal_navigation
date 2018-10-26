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
#include "verbal_navigation/MapInfo.h"
#include "verbal_navigation/Preposition.h"
#include "verbal_navigation/VerbPhrase.h"


const std::string projectDir = "/home/fri/TGI_FRIdays_ws/";



int main (int argc, char** argv) {

	// FuturePoseStamped initialPose;
	// FuturePoseStamped goalPose;
	ros::init(argc, argv, "main_node");
	ros::NodeHandle n;

	geometry_msgs::PoseStamped initialPose;
	geometry_msgs::PoseStamped goalPose;

	initialPose.header.stamp = ros::Time::now();
	initialPose.header.frame_id = "/level_mux_map";
	initialPose.pose.position.x = 14.6793708801;
	initialPose.pose.position.y = 110.153694153;
	initialPose.pose.position.z = 0.0;
	initialPose.pose.orientation.x = 0.0;
	initialPose.pose.orientation.y = 0.0;
	initialPose.pose.orientation.z = -0.04113175032;
	initialPose.pose.orientation.w = 0.999153731473;

	goalPose.header.stamp = ros::Time::now();
	goalPose.header.frame_id = "/level_mux_map";
	goalPose.pose.position.x = 47.8235244751;
	goalPose.pose.position.y = 108.96232605;
	goalPose.pose.position.z = 0.0;
	goalPose.pose.orientation.x = 0.0;
	goalPose.pose.orientation.y = 0.0;
	goalPose.pose.orientation.z = -0.65703277302;
	goalPose.pose.orientation.w = 0.753862013354;


	// ros::Subscriber sub = n.subscribe("/initialpose", 100, &FuturePoseStamped::setFromPoseWithCovarianceStamped, &initialPose);
	// ros::Subscriber sub1 = n.subscribe("/move_base_interruptable_simple/goal", 100, &FuturePoseStamped::setFromPoseStamped, &goalPose);

	ros::param::set("~map_file", projectDir +  "src/verbal_navigation/src/3ne/3ne.yaml");
	ros::param::set("~data_directory", projectDir + "src/verbal_navigation/src/3ne");
	ros::param::set("move_base/orientation_mode", 1);
	bwi_logical_translator::BwiLogicalTranslator translator;
	translator.initialize();


	// while(!(initialPose.isAvailable() && goalPose.isAvailable()) && ros::ok()) ros::spinOnce();

	ROS_INFO("HAVE START AND GOAL");
	//ros::ServiceClient path_client = n.serviceClient <nav_msgs::GetPlan> ("move_base/NavfnROS/make_plan");
	ros::ServiceClient path_client = n.serviceClient <nav_msgs::GetPlan> ("move_base/make_plan");
	path_client.waitForExistence();
	ROS_INFO("Path client found");

	nav_msgs::GetPlan srv;
	// srv.request.start = initialPose.getPose();
	// srv.request.goal = goalPose.getPose();
	srv.request.start = initialPose;
	srv.request.goal = goalPose;

	srv.request.tolerance = -1.0f;

	// MAKE THE PLAN, which populates the response with a list of poses
	path_client.call(srv);

	ROS_INFO("Response received, size %d", srv.response.plan.poses.size());
	std::vector<geometry_msgs::PoseStamped> pose_list = srv.response.plan.poses;


	// do the heavy lifting in this class
	MapInfo MapInfo(translator, pose_list);




	while (ros::ok()) {
		ros::spinOnce();
	}
}
