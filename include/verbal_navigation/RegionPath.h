#ifndef REGION_PATH
#define REGION_PATH

#include "ros/ros.h"
#include <geometry_msgs/PoseStamped.h>
#include <bwi_logical_translator/bwi_logical_translator.h>
#include <bwi_mapper/structures/point.h>
#include <vector>
#include <string>
#include <map>

class RegionPath {
  std::vector<std::string> pathOfRegions;
  std::map<std::string, std::vector<geometry_msgs::PoseStamped>> pointsInRegions;

  std::string getRegion(bwi_logical_translator::BwiLogicalTranslator& translator, geometry_msgs::PoseStamped currentLocation);

public:
  RegionPath(bwi_logical_translator::BwiLogicalTranslator& translator, std::vector<geometry_msgs::PoseStamped> path);
};
#endif
