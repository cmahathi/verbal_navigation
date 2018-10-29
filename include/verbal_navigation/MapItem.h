#ifndef LANDMARK
#define LANDMARK

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Float64.h>
#include <string>
#include <cmath>

class MapItem {
  std::string name;
  geometry_msgs::Pose pose;
  std::string common_name;

public:
  MapItem(std::string name) : name(name) { }
  MapItem(std::string name, geometry_msgs::Pose pose) : name(name), pose(pose) { }
  std_msgs::Float64 distanceTo(geometry_msgs::Pose dest);
  std::string getName();
  geometry_msgs::Pose getPose();
  void setCommonName(std::string common_name);
  std::string getCommonName();

};
#endif
