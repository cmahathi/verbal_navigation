#ifndef MAP_INFO
#define MAP_INFO

#include "ros/ros.h"
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Float64.h>
#include <bwi_logical_translator/bwi_logical_translator.h>
#include <bwi_mapper/structures/point.h>
#include <tf/tf.h>
#include <vector>
#include <string>
#include <map>
#include <cmath>
#include <limits>
#include <Eigen/Dense>

#include <memory>
#include "verbal_navigation/Predicates.h"
#include "verbal_navigation/MapItem.h"
#include "verbal_navigation/RegionPath.h"



class MapInfo {

  // user-specified destination Name. Should be a door or landmark
  std::string destinationCommonName;

  // bwi translator containing important helper methods
  bwi_logical_translator::BwiLogicalTranslator translator;

  // ordered list of poses returned by the translator from start to dest
  std::vector<geometry_msgs::PoseStamped> poseList;

  // ordered list of the names of regions that the path traverses
  std::vector<std::string> regionList;

  // keys: region names; values: list of points in that region
  std::map<std::string, std::vector<geometry_msgs::PoseStamped>> regionToPosesMap;

  // keys: region names; values: 2D vector representing orientation or region
  std::map<std::string, Eigen::Vector2d> regionToOrientationMap;

  // list of all landmarks as "MapItem" objects
  std::vector<MapItem> landmarkList;

  // Instruction objects used to generate natural language
  std::vector<std::shared_ptr<Instruction>> instructionList;

  // keys: region names; values: list of "MapItem" objects representing landmarks in that region
  std::map<std::string, std::vector<MapItem>> regionToMapItemsMap;
  std::map<std::string, std::string> labelToCommonNameMap;

  // Contains the ordered path of regions and the floor those regions are on
  RegionPath regions;

  // a string representing the natural language instruction for this path.
  std::string directions;

  bool readAttributesFile(const std::string& filename);
  void populateRegionAttributes();
  void buildRegionAndPointsInfo();
  void buildRegionOrientationInfo();
  void buildRegionsToMapItemsMap();
  void buildInstructions();


  /* HLPER METHODS */
  bool mapItemInRegion(std::string region, MapItem item);
  MapItem getClosestLandmarkTo(geometry_msgs::PoseStamped pose);
  Directions getDirectionBetween(Region fromRegion, Region toRegion);
  Directions getFinalDirection(Region finalRegion);
  std::string getRegion(geometry_msgs::Pose currentLocation);


public:
  // A landmark must be <= DISTANCE_THRESHOLD away from a turn point for that landmark to be used in the turn instruction
  static constexpr double DISTANCE_THRESHOLD = 3;
  // 2 regions must have orientation difference >= ANGLE_THRESHOLD for a turn instruction to be created
  static constexpr double ANGLE_THRESHOLD = M_PI/6;

  MapInfo(bwi_logical_translator::BwiLogicalTranslator& translator, std::vector<geometry_msgs::PoseStamped> path, std::string dest, std::string floor);
  std::string generateDirections();
  static std_msgs::Float64 distanceBetween(geometry_msgs::Pose firstPose, geometry_msgs::Pose lastPose);
  RegionPath getRegionPath();
};
#endif
