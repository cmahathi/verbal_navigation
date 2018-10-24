#include "verbal_navigation/MapInfo.h"

//Constructor
MapInfo::MapInfo(bwi_logical_translator::BwiLogicalTranslator& trans, std::vector<geometry_msgs::PoseStamped> path)
  : translator(trans), poseList(path), regionList(), regionToPointsMap() {

  if(poseList.empty()) {
    ROS_INFO("ERROR: Initialized with empty path");
  }
  buildMapInfo();
  //regionsToAvgOrientation(regionToPointsMap);
}

void MapInfo::buildMapInfo() {
  //adds the first region the path goes through
  regionList.push_back(getRegion(translator, poseList.front()));
  ROS_INFO("Added region: %s\n", getRegion(translator, poseList.front()).c_str());
  //adding a region the path goes through
  for(size_t i = 0; i < poseList.size(); ++i) {
    //geometry_msgs::PoseStamped
    auto currentLocation = poseList[i];
    ROS_INFO("Orientation: %lf, %lf, %lf, %lf", currentLocation.pose.orientation.x,currentLocation.pose.orientation.y,currentLocation.pose.orientation.z,currentLocation.pose.orientation.w);

    std::string region = getRegion(translator, currentLocation);

    //only adds the region to the list of regions if the previous pose was
    //not in the region
    if (region.compare("") != 0) {
      if( (regionList.back()).compare(region) != 0) {
        regionList.push_back(region);
        ROS_INFO("Added region: %s\n", region.c_str());
      }
      //maps points to regions
      auto points = regionToPointsMap.find(region);
      if(points == regionToPointsMap.end()) {
        std::vector<geometry_msgs::PoseStamped> newList;
        newList.push_back(currentLocation);
        regionToPointsMap.emplace(std::make_pair(region, newList));
      } else {
        points->second.push_back(currentLocation);
      }
    }
  }
}

//takes in a location on the map and returns the region it is in
std::string MapInfo::getRegion(
  bwi_logical_translator::BwiLogicalTranslator& translator,
  geometry_msgs::PoseStamped currentLocation) {
  float robot_x = currentLocation.pose.position.x;
  float robot_y = currentLocation.pose.position.y;

  bwi_mapper::Point2f mapPoint(robot_x, robot_y);
  return translator.getLocationString(translator.getLocationIdx(mapPoint));
}

void MapInfo::regionsToAvgOrientation(
  std::map<std::string, std::vector<geometry_msgs::PoseStamped>> map) {
    std::map<std::string, float> result;
    double yawTotal = 0.0;
    int numPoints = 0;
    for (int i = 0; i < regionList.size(); i++) {
      ROS_INFO("Getting new pose list");
      std::string region = regionList.at(i);
      if (regionToPointsMap.find(region) != regionToPointsMap.end()) {
        std::vector<geometry_msgs::PoseStamped> poseList = regionToPointsMap.find(region)->second;
        ROS_INFO("Region: %s", region.c_str());
        for (int i = 0; i < poseList.size(); i++) {
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
    // else
    //   ROS_INFO("Invalid region detected: %s", region.c_str());
  }
}

std::map<std::string, std::vector<geometry_msgs::PoseStamped>> MapInfo::getRegionToPointsMap(){
  return regionToPointsMap;
}

std::vector<std::string> MapInfo::getRegionList(){
  return regionList;
}
