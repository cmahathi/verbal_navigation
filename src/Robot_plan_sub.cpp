#include <stdio.h>
#include <string.h>
#include "std_msgs/String.h"
#include "ros/ros.h"
#include "verbal_navigation/Wavenet.h"
#include <sound_play/sound_play.h>
#include <unistd.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
MoveBaseClient ac;
ros::ServiceClient client;

void planCallback(const verbal_navigation::Path &msg){

   //do something with the robot id 
   std::string action_type = msg->action_type;
   if (action_type.compare("initialize") == 0){
      geometry_msgs::Pose init_pose = msg->initial_pose;
      int success = goto_location(init_pose);
   }
   else  (action_type.compare("L") == 0){
      //"Follow me"
      geometry_msgs::Pose final_pose = msg->end_pose;
      int success = goto_location(final_pose);
   }
   else if (action_type.compare("I") == 0){
      verbal_navigation::Wavenet srv;
      srv.request.a = msg->instructions;
   }
}

int goto_location(geometry_msgs::Pose dest_pose){
      move_base_msgs::MoveBaseGoal goal;
      goal.target_pose.header.frame_id = "base_link";
      goal.target_pose.header.stamp = ros::Time::now();
      goal.target_pose.pose = init_pose;
      ac.sendGoal(goal);
      ac.waitForResult();

      return ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED;
   }
}
int main(int argc, char **argv){
   ros::Subscriber plan_sub = n.subscribe("Plan", 1000, planCallback);

   ros::init(argc, argv, "robot_navigation_goals");
   MoveBaseClient ac("move_base", true);
   while(!ac.waitForServer(ros::Duration(5.0))){
      ROS_INFO("Waiting for the move_base action server to come up");
   }

   ros::NodeHandle n;
   client = n.serviceClient<verbal_navigation::Wavenet>("wavenet");
   
}
