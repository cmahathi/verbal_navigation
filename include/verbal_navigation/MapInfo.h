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
#include <cmath>
#include <limits>
#include <Eigen/Dense>

#include "verbal_navigation/VerbPhrase.h"
#include "verbal_navigation/Preposition.h"
#include "verbal_navigation/Landmark.h"



class MapInfo {
  bwi_logical_translator::BwiLogicalTranslator translator;

  std::vector<geometry_msgs::PoseStamped> poseList;
  std::vector<std::string> regionList; //list of regions path goes through in order (?)
  std::map<std::string, std::vector<geometry_msgs::PoseStamped>> regionToPosesMap; //maps a list of points (in order?) to each region

  std::map<std::string, Eigen::Vector2d> regionToOrientationMap;

  std::vector<Landmark> landmarkList; // list of all landmarks
  std::map<std::string, std::vector<Landmark>> regionToLandmarksMap; // maps region names to vector of landmarks names within that region

  std::vector<VerbPhrase> instructionList; // Instruction objects used to generate natural language

  void buildRegionAndPointsInfo(); //
  void buildRegionOrientationInfo();
  void buildRegionsToLandmarksMap();
  void buildInstructions();

  /* HLPER METHODS */
  Landmark getClosestLandmarkTo(geometry_msgs::PoseStamped pose);
  Directions shouldTurnBetween(std::string fromRegion, std::string toRegion);


public:
  static constexpr double DISTANCE_THRESHOLD = 3;
  static constexpr double ANGLE_THRESHOLD = M_PI/4;

  MapInfo(bwi_logical_translator::BwiLogicalTranslator& translator, std::vector<geometry_msgs::PoseStamped> path);

  std::string getRegion(geometry_msgs::Pose currentLocation);
  // std::vector<std::string> getRegionList();
  // std::map<std::string, std::vector<geometry_msgs::PoseStamped>> getregionToPosesMap();
};
#endif
