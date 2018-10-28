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
#include "verbal_navigation/Arrival.h"
#include <memory>
#include "verbal_navigation/Preposition.h"
#include "verbal_navigation/Landmark.h"



class MapInfo {

  // bwi translator containing important helper methods
  bwi_logical_translator::BwiLogicalTranslator translator;

  // ordered list of poses returned by the translator from start to dest
  std::vector<geometry_msgs::PoseStamped> poseList;

  // ordered list of regions that the path traverses
  std::vector<std::string> regionList;

  // keys: region names; values: list of points in that region
  std::map<std::string, std::vector<geometry_msgs::PoseStamped>> regionToPosesMap;

  // keys: region names; values: 2D vector representing orientation or region
  std::map<std::string, Eigen::Vector2d> regionToOrientationMap;

  // list of all landmarks as "Landmark" objects
  std::vector<Landmark> landmarkList;

  // Instruction objects used to generate natural language
  std::vector<std::shared_ptr<Instruction>> instructionList; 

  // keys: region names; values: list of "Landmark" objects representing landmarks in that region
  std::map<std::string, std::vector<Landmark>> regionToLandmarksMap;

  // a string representing the natural language instruction for this path.
  std::string directions;

  void buildRegionAndPointsInfo();
  void buildRegionOrientationInfo();
  void buildRegionsToLandmarksMap();
  void buildInstructions();


  /* HLPER METHODS */
  Landmark getClosestLandmarkTo(geometry_msgs::PoseStamped pose);
  Directions getDirectionBetween(std::string fromRegion, std::string toRegion);
  std::string getRegion(geometry_msgs::Pose currentLocation);


public:
  // A landmark must be <= DISTANCE_THRESHOLD away from a turn point for that landmark to be used in the turn instruction
  static constexpr double DISTANCE_THRESHOLD = 3;
  // 2 regions must have orientation difference >= ANGLE_THRESHOLD for a turn instruction to be created
  static constexpr double ANGLE_THRESHOLD = M_PI/4;

  MapInfo(bwi_logical_translator::BwiLogicalTranslator& translator, std::vector<geometry_msgs::PoseStamped> path);

  void generateDirections();
};
#endif
