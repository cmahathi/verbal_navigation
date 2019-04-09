#include <stdio.h>
#include <string.h>
#include "std_msgs/String.h"
#include "ros/ros.h"
#include "verbal_navigation/Wavenet.h"
#include <sound_play/sound_play.h>
#include <unistd.h>
#include "visualization_msgs/Marker.h"
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include "verbal_navigation/Robot_Action.h"

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
MoveBaseClient* move_client;
ros::Publisher vis_pub;
ros::ServiceClient speech_client;

int goto_location(geometry_msgs::Pose dest_pose);

void planCallback(const verbal_navigation::Robot_Action& msg) {

   //do something with the robot id 
   std::string action_type = msg.action_type;
   if (action_type.compare("initialize") == 0){
      geometry_msgs::Pose init_pose = msg.initial_pose;
      int success = goto_location(init_pose);
   }
   else if (action_type.compare("L") == 0){
      verbal_navigation::Wavenet srv;
      srv.request.text = "Follow me!";
      ROS_INFO("Speaking: Follow me!");
      //speech_client.call(srv);

      geometry_msgs::Pose final_pose = msg.end_pose;
      int success = goto_location(final_pose);
   }
   else if (action_type.compare("I") == 0){
      verbal_navigation::Wavenet srv;
      srv.request.text = msg.instructions;
      ROS_INFO("Speaking: %s", msg.instructions.c_str());
      //speech_client.call(srv);
   }
   else if (action_type.compare("T") == 0) {
      verbal_navigation::Wavenet srv;
      std::string txt = msg.instructions;
      txt.append(" My robotic colleague will meet you there.");
      srv.request.text = txt;
      ROS_INFO("Speaking: %s", txt.c_str());
      //speech_client.call(srv);
   }
}

int goto_location(geometry_msgs::Pose dest_pose) {
   move_base_msgs::MoveBaseGoal goal;
   goal.target_pose.header.frame_id = "base_link";
   goal.target_pose.header.stamp = ros::Time::now();
   goal.target_pose.pose = dest_pose;

   ROS_INFO("Moving to goal");
   // move_client->sendGoal(goal);
   // move_client->waitForResult();

   //return move_client->getState() == actionlib::SimpleClientGoalState::SUCCEEDED;
   return 1;
}

void display_arrow () {
   ROS_INFO("Drawing arrow");
   visualization_msgs::Marker marker;
    marker.header.frame_id = "/map";
    marker.header.stamp = ros::Time::now();

    // Set the namespace and id for this marker.  This serves to create a unique ID
    // Any marker sent with the same namespace and id will overwrite the old one
    marker.ns = "basic_shapes";
    marker.id = 0;

    // Set the marker type.  Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
    marker.type = visualization_msgs::Marker::ARROW;

    // Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
    marker.action = visualization_msgs::Marker::ADD;

    // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
    marker.pose.position.x = 0;
    marker.pose.position.y = 0;
    marker.pose.position.z = 0;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;

    // Set the scale of the marker -- 1x1x1 here means 1m on a side
    marker.scale.x = 1.0;
    marker.scale.y = 1.0;
    marker.scale.z = 1.0;

    // Set the color -- be sure to set alpha to something non-zero!
    marker.color.r = 0.0f;
    marker.color.g = 1.0f;
    marker.color.b = 0.0f;
    marker.color.a = 1.0;

    marker.lifetime = ros::Duration();
   while (ros::ok()) {
      vis_pub.publish( marker );
   }
}

int main(int argc, char **argv){
   ros::init(argc, argv, "RobotPlanExecutor");
   ros::NodeHandle n("~");
   ros::Subscriber plan_sub = n.subscribe("/robot_plan", 1000, planCallback);

   vis_pub = n.advertise<visualization_msgs::Marker>( "visualization_marker", 0 );
   ROS_INFO("Drawing arrow");
   // Set the frame ID and timestamp.  See the TF tutorials for information on these.
   display_arrow();


   MoveBaseClient mc("move_base", true);
   move_client = &mc;
   // while(!move_client->waitForServer(ros::Duration(5.0))){
   //    ROS_INFO("Waiting for the move_base action server to come up");
   // }
   ROS_INFO("MoveBase Client Found!");


   speech_client = n.serviceClient<verbal_navigation::Wavenet>("/wavenet");
   //speech_client.waitForExistence();
	ROS_INFO("Speech Client Found!");


   ros::spin();

}
