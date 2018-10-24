/*
*
*/
#ifndef MAP_INFO
#define MAP_INFO

#include "ros/ros.h"
#include <geometry_msgs/PoseStamped.h>
#include <bwi_logical_translator/bwi_logical_translator.h>
#include <bwi_mapper/structures/point.h>
#include <tf/tf.h>
#include <vector>
#include <string>
#include <map>

class MapInfo {
  std::vector<std::string> regionList; //list of regions path goes through in order (?)
  std::map<std::string, std::vector<geometry_msgs::PoseStamped>> regionToPointsMap; //maps a list of points (in order?) to each region
  bwi_logical_translator::BwiLogicalTranslator translator;
  std::vector<geometry_msgs::PoseStamped> poseList;
  void buildMapInfo();
  void regionsToAvgOrientation(std::map<std::string, std::vector<geometry_msgs::PoseStamped>> map);

public:
  MapInfo(bwi_logical_translator::BwiLogicalTranslator& translator, std::vector<geometry_msgs::PoseStamped> path);
  std::string getRegion(bwi_logical_translator::BwiLogicalTranslator& translator, geometry_msgs::PoseStamped currentLocation);
  std::vector<std::string> getRegionList();
  std::map<std::string, std::vector<geometry_msgs::PoseStamped>> getRegionToPointsMap();



};
#endif
