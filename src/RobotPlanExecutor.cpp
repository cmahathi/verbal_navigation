#include <stdio.h>
#include <string.h>
#include "std_msgs/String.h"
#include "ros/ros.h"
#include "verbal_navigation/Wavenet.h"
#include <sound_play/sound_play.h>
#include <unistd.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include "verbal_navigation/Robot_Action.h"

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
MoveBaseClient* move_client;
ros::ServiceClient speech_client;
ros::Publisher test_pub;
int goto_location(geometry_msgs::Pose dest_pose);

void planCallback(const verbal_navigation::Robot_Action& msg) {

   //do something with the robot id 
   if (msg.robot_id.compare("r2") == 0) {
   std::string action_type = msg.action_type;
   if (action_type.compare("initialize") == 0){
      geometry_msgs::Pose init_pose = msg.initial_pose;
      int success = goto_location(init_pose);
   }
   else if (action_type.compare("L") == 0){
      verbal_navigation::Wavenet srv;
      srv.request.text = "Follow me!";
      ROS_INFO("Speaking: Follow me!");
      speech_client.call(srv);

      geometry_msgs::Pose final_pose = msg.end_pose;
      int success = goto_location(final_pose);
   }
   else if (action_type.compare("I") == 0){
      verbal_navigation::Wavenet srv;
      srv.request.text = msg.instructions;
      ROS_INFO("Speaking: %s", msg.instructions.c_str());
      speech_client.call(srv);
   }
   else if (action_type.compare("T") == 0) {
      verbal_navigation::Wavenet srv;
      std::string txt = msg.instructions;
      txt.append(" My robotic colleague will meet you there.");
      srv.request.text = txt;
      ROS_INFO("Speaking: %s", txt.c_str());
      speech_client.call(srv);
   }
   }
}

int goto_location(geometry_msgs::Pose dest_pose) {
   move_base_msgs::MoveBaseGoal goal;
   goal.target_pose.header.frame_id = "level_mux_map";
   goal.target_pose.header.stamp = ros::Time::now();
   goal.target_pose.pose = dest_pose;
   
   //test_pub.publish(goal.target_pose);
   ROS_INFO("Moving to goal");
   move_client->sendGoal(goal);
   move_client->waitForResult();

   return move_client->getState() == actionlib::SimpleClientGoalState::SUCCEEDED;
}

int main(int argc, char **argv){
   ros::init(argc, argv, "RobotPlanExecutor");
   ros::NodeHandle n("~");
   ros::Subscriber plan_sub = n.subscribe("/robot_plan", 1000, planCallback);
   test_pub = n.advertise<geometry_msgs::PoseStamped>("/test_pose", 100);
   MoveBaseClient mc("move_base", true);
   move_client = &mc;
   while(!move_client->waitForServer(ros::Duration(5.0))){
      ROS_INFO("Waiting for the move_base action server to come up");
   }
   ROS_INFO("MoveBase Client Found!");


   speech_client = n.serviceClient<verbal_navigation::Wavenet>("/wavenet");
   speech_client.waitForExistence();
	ROS_INFO("Speech Client Found!");

   ros::spin();
   
}
