/*
*
*/
#ifndef REGION_PATH
#define REGION_PATH

#include "ros/ros.h"
#include <geometry_msgs/PoseStamped.h>
#include <bwi_logical_translator/bwi_logical_translator.h>
#include <bwi_mapper/structures/point.h>
#include <tf/tf.h>
#include <vector>
#include <string>
#include <map>

class RegionPath {
  std::vector<std::string> pathOfRegions; //list of regions path goes through in order (?)
  std::map<std::string, std::vector<geometry_msgs::PoseStamped>> pointsInRegions; //maps a list of points (in order?) to each region
  //
  // std::string getRegion(bwi_logical_translator::BwiLogicalTranslator& translator, geometry_msgs::PoseStamped currentLocation);


public:
  RegionPath(bwi_logical_translator::BwiLogicalTranslator& translator,
    std::vector<geometry_msgs::PoseStamped> path);
  std::string getRegion(bwi_logical_translator::BwiLogicalTranslator& translator,
     geometry_msgs::PoseStamped currentLocation);
  std::vector<std::string> getPathOfRegions();
  std::map<std::string, std::vector<geometry_msgs::PoseStamped>> getPointsInRegions();
  std::map<std::string, float> regionsToAvgOrientation(
    std::map<std::string, std::vector<geometry_msgs::PoseStamped>> map);
};
#endif
