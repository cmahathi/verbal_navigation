#include "verbal_navigation/RegionPath.h"

RegionPath::RegionPath(bwi_logical_translator::BwiLogicalTranslator& translator, std::vector<geometry_msgs::PoseStamped> pathOfPoses) : pathOfRegions(), pointsInRegions() {
  if(pathOfPoses.empty()) {
    ROS_INFO("ERROR: Initialized with empty path");
  }
  pathOfRegions.push_back(getRegion(translator, pathOfPoses.front()));
  for(size_t i = 0; i < pathOfPoses.size(); ++i) {
    auto currentLocation = pathOfPoses.at(i);
    auto region = getRegion(translator, currentLocation);

    if(pathOfRegions.back().compare(region) != 0) {
      pathOfRegions.push_back(region);
      ROS_INFO("Added region: %s\n", region.c_str());
    }

    auto points = pointsInRegions.find(region);
    if(points == pointsInRegions.end()) {
      std::vector<geometry_msgs::PoseStamped> newList;
      newList.push_back(currentLocation);
      pointsInRegions.emplace(std::make_pair(region, newList));
    } else {
      points->second.push_back(currentLocation);
    }
  }
}


std::string RegionPath::getRegion(bwi_logical_translator::BwiLogicalTranslator& translator, geometry_msgs::PoseStamped currentLocation) {
  float robot_x = currentLocation.pose.position.x;
  float robot_y = currentLocation.pose.position.y;

  bwi_mapper::Point2f mapPoint(robot_x, robot_y);
  return translator.getLocationString(translator.getLocationIdx(mapPoint));
  // return translator.getRegionString(translator.getRegionIdx(mapPoint));
}
