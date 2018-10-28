#include "verbal_navigation/Landmark.h"

std_msgs::Float64 Landmark::distanceTo(geometry_msgs::Pose dest) {
  auto dx = pose.position.x - dest.position.x;
  auto dy = pose.position.y- dest.position.y;

  std_msgs::Float64 distance;
  distance.data = std::sqrt(std::pow(dx, 2) + std::pow(dy, 2));
  return distance;
}

std::string Landmark::getName() {
  return name;
}

geometry_msgs::Pose Landmark::getPose() {
  return pose;
}
