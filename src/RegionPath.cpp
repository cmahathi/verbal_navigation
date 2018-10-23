#include "verbal_navigation/RegionPath.h"

//Constructor
RegionPath::RegionPath(bwi_logical_translator::BwiLogicalTranslator& translator,
  std::vector<geometry_msgs::PoseStamped> pathOfPoses)
  : pathOfRegions(), pointsInRegions() {

  if(pathOfPoses.empty()) {
    ROS_INFO("ERROR: Initialized with empty path");
  }

  //adds the first region the path goes through
  pathOfRegions.push_back(getRegion(translator, pathOfPoses.front()));
  ROS_INFO("Added region: %s\n", getRegion(translator, pathOfPoses.front()).c_str());
  //adding a region the path goes through
  for(size_t i = 0; i < pathOfPoses.size(); ++i) {
    auto currentLocation = pathOfPoses.at(i);
    std::string region = getRegion(translator, currentLocation);

    //only adds the region to the list of regions if the previous pose was
    //not in the region
    if (region.compare("") != 0) {
      if( (pathOfRegions.back()).compare(region) != 0) {
        pathOfRegions.push_back(region);
        ROS_INFO("Added region: %s\n", region.c_str());
      }

      //maps points to regions
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
  //regionsToAvgOrientation(pointsInRegions);
}

//takes in a location on the map and returns the region it is in
std::string RegionPath::getRegion(
  bwi_logical_translator::BwiLogicalTranslator& translator,
  geometry_msgs::PoseStamped currentLocation) {
  float robot_x = currentLocation.pose.position.x;
  float robot_y = currentLocation.pose.position.y;

  bwi_mapper::Point2f mapPoint(robot_x, robot_y);
  return translator.getLocationString(translator.getLocationIdx(mapPoint));
}

std::map<std::string, float> RegionPath::regionsToAvgOrientation(
  std::map<std::string, std::vector<geometry_msgs::PoseStamped>> map) {
    std::map<std::string, float> result;
    double yawTotal = 0.0;
    int numPoints = 0;
    for (int i = 0; i < pathOfRegions.size(); i++) {
      ROS_INFO("Getting new pose list");
      std::string region = pathOfRegions.at(i);
      if (pointsInRegions.find(region) != pointsInRegions.end()) {
        std::vector<geometry_msgs::PoseStamped> poseList = pointsInRegions.find(region)->second;
        ROS_INFO("Region: %s", region.c_str());
        for (int i = 0; i < poseList.size(); i++) {
  //ROS_INFO
          geometry_msgs::PoseStamped pose = poseList.at(i);
          ROS_INFO("Position: %lf %lf %lf", pose.pose.position.x, pose.pose.position.y, pose.pose.position.z);
          ROS_INFO("Orietation: %lf %lf %lf %lf", pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z, pose.pose.orientation.w);
          tf::Quaternion q(
            pose.pose.orientation.x,
            pose.pose.orientation.y,
            pose.pose.orientation.z,
            pose.pose.orientation.w);
            tf::Matrix3x3 m(q);
         double roll, pitch, yaw;

         m.getRPY(roll, pitch, yaw);
         ROS_INFO ("ROLL: %lf, PITCH: %lf, YAW: %lf", roll, pitch, yaw);
        }
    }
    }
}

std::map<std::string, std::vector<geometry_msgs::PoseStamped>> RegionPath::getPointsInRegions(){
  return pointsInRegions;
}

std::vector<std::string> RegionPath::getPathOfRegions(){
  return pathOfRegions;
}
