#include "verbal_navigation/MapItem.h"

// Euclidean distance between this landmark and specified point
std_msgs::Float64 MapItem::distanceTo(geometry_msgs::Pose dest) {
  auto dx = pose.position.x - dest.position.x;
  auto dy = pose.position.y- dest.position.y;
  std_msgs::Float64 distance;
  distance.data = std::sqrt(std::pow(dx, 2) + std::pow(dy, 2));
  return distance;
}


std::string MapItem::getName() {
  return name;
}

std::string MapItem::getCommonName() {
  if (common_name == "")
    return name;
  return common_name;
}

void MapItem::setCommonName(std::string cn) {
  common_name = cn;
}


geometry_msgs::Pose MapItem::getPose() {
  return pose;
}
