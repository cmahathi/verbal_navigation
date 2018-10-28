#ifndef LANDMARK
#define LANDMARK

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Float64.h>
#include <string>
#include <cmath>

class Landmark {
  std::string name;
  geometry_msgs::Pose pose;

public:
  Landmark(std::string name, geometry_msgs::Pose pose) : name(name), pose(pose) { }
  std_msgs::Float64 distanceTo(geometry_msgs::Pose dest);
  std::string getName();
  geometry_msgs::Pose getPose();
};

#endif
