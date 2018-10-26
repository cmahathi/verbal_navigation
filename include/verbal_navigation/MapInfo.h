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
#include "verbal_navigation/VerbPhrase.h"

#include <Eigen/Dense>




class MapInfo {
  bwi_logical_translator::BwiLogicalTranslator translator;

  std::vector<geometry_msgs::PoseStamped> poseList;
  std::vector<std::string> regionList; //list of regions path goes through in order (?)
  std::map<std::string, std::vector<geometry_msgs::PoseStamped>> regionToPointsMap; //maps a list of points (in order?) to each region

  std::map<std::string, Eigen::Vector3d> regionToOrientationMap;

  std::map<std::string, geometry_msgs::Pose> landmarkNameToPositionMap; // maps landmark names to landmark locations
  std::map<std::string, std::vector<std::string>> regionToLandmarksMap; // maps region names to vector of landmarks names within that region

  std::vector<Instruction> instructionList; // Instruction objects used to generate natural language

  void buildRegionAndPointsInfo(); //
  void buildRegionOrientationInfo();
  void buildRegionsToLandmarksMap();
  void buildInstructions();


public:
  MapInfo(bwi_logical_translator::BwiLogicalTranslator& translator, std::vector<geometry_msgs::PoseStamped> path);

  std::string getRegion(geometry_msgs::Pose currentLocation);
  // std::vector<std::string> getRegionList();
  // std::map<std::string, std::vector<geometry_msgs::PoseStamped>> getRegionToPointsMap();





};
#endif
