#include "verbal_navigation/MapInfo.h"
#include <boost/filesystem.hpp>
#include <fstream>
#include <libgen.h>
#include <stdexcept>
#include <stdio.h>
#include <yaml-cpp/yaml.h>

// constructor
MapInfo::MapInfo(bwi_logical_translator::BwiLogicalTranslator& trans, std::vector<geometry_msgs::PoseStamped> path)
  : translator(trans), poseList(path) {

  if(poseList.empty()) {
    ROS_INFO("ERROR: Initialized with empty path");
  }

  readCommonNamesFile(boost::filesystem::current_path().string() + "/src/3ne/common_names.yaml");
  buildRegionAndPointsInfo();
  buildRegionOrientationInfo();
  buildRegionsToMapItemsMap();
  buildInstructions();
}


// turns path of poses into list of regions, and builds map of regions to poses
void MapInfo::buildRegionAndPointsInfo() {

  // add the robot's initial region to the regionList
  regionList.push_back(getRegion(poseList.front().pose));
  //ROS_INFO("Added region: %s\n", getRegion(translator, poseList.front()).c_str());
  ROS_INFO("START REGIONS");
  // add each point's region to the region list, and add the point to the regionToPosesMap
  for(size_t i = 0; i < poseList.size(); ++i) {
    auto currentLocation = poseList[i];
    //ROS_INFO("Orientation: %lf, %lf, %lf, %lf", currentLocation.pose.orientation.x,currentLocation.pose.orientation.y,currentLocation.pose.orientation.z,currentLocation.pose.orientation.w);

    std::string region = getRegion(currentLocation.pose);

    // only add region to regionList if it's not same as last seen region
    if (region.compare("") != 0) {
      if( (regionList.back()).compare(region) != 0) {
        regionList.push_back(region);
        ROS_INFO("REGION: %s", region.c_str());
      }
      // add point to regionToPosesMap
      auto points = regionToPosesMap.find(region);
      if(points == regionToPosesMap.end()) {
        std::vector<geometry_msgs::PoseStamped> newList;
        newList.push_back(currentLocation);
        regionToPosesMap.emplace(std::make_pair(region, newList));
      } else {
        points->second.push_back(currentLocation);
      }
    }
  }
  ROS_INFO("END REGIONS");

}


// determines representative orientation for each region using first and last pose in that region
void MapInfo::buildRegionOrientationInfo() {
  // calculate representative orientation for every region
  std::map<std::string, std::vector<geometry_msgs::PoseStamped>>::iterator it;
  for ( it = regionToPosesMap.begin(); it != regionToPosesMap.end(); it++ ) {

    std::vector<geometry_msgs::PoseStamped> posesInRegionList = it->second;
    auto x = posesInRegionList.back().pose.position.x - posesInRegionList.front().pose.position.x;
    auto y = posesInRegionList.back().pose.position.y - posesInRegionList.front().pose.position.y;
    Eigen::Vector2d regionPathVector(x, y);

    regionToOrientationMap.emplace(std::make_pair(it->first, regionPathVector));
    // ROS_INFO("r2o %s %lf %lf", regionToOrientationMap.find(it->first)->first.c_str(), regionPathVector(0), regionPathVector(1));
  }
}


// builds a "MapItem" object for each landmark; builds a list of landmarks;
// build a map of regions to landmarks in that region
void MapInfo::buildRegionsToMapItemsMap() {

  // get a map of landmark names to landmark positions from the translator
  const auto& landmarkNameToPositionMap = translator.getObjectApproachMap();
  for (auto const& pair : landmarkNameToPositionMap) {
    MapItem landmark(pair.first, pair.second);
    landmarkList.push_back(landmark);

    std::string region = getRegion(landmark.getPose());

    auto regionToMapItemsPair = regionToMapItemsMap.find(region);

    // if region doesn't have any landmarks yet, add this first one
    if(regionToMapItemsPair == regionToMapItemsMap.end()) {
      std::vector<MapItem> newList;
      newList.push_back(landmark);
      regionToMapItemsMap.emplace(std::make_pair(region, newList));
    } else {
      // if the region already exists in this map, just add this landmark to the region's value
      regionToMapItemsPair->second.push_back(landmark);
    }
  }

  // CODE TO PRINT
  // std::map<std::string, std::vector<std::string>>::iterator it;
  // for (auto it = regionToMapItemsMap.begin(); it != regionToMapItemsMap.end(); it++ ) {
  //   ROS_INFO("Region Name: %s", it->first.c_str());
  //   for (auto landmark : it->second) {
  //     ROS_INFO("  MapItem: %s", landmark.getName().c_str());
  //   }
  // }
}


// builds a list of predicates representing the verbal instructions for this path
void MapInfo::buildInstructions() {

  // iterate through all the regions except the last one
  for (int ix = 0; ix < regionList.size() - 1; ix ++) {
    std::string thisRegion = regionList[ix];
    std::string nextRegion = regionList[ix + 1];
    std::string thisRegionName = labelToCommonNameMap[thisRegion];
    std::string nextRegionName = labelToCommonNameMap[nextRegion];

    auto direction = getDirectionBetween(thisRegion, nextRegion);

    auto travelIns = std::make_shared<VerbPhrase>("travel");
    travelIns->setStartRegion(thisRegionName);
    travelIns->setEndRegion(thisRegionName);
    MapItem regionItem(thisRegion);
    regionItem.setCommonName(labelToCommonNameMap[regionItem.getName()]);
    travelIns->addPreposition(Preposition("through", regionItem));
    instructionList.push_back(travelIns);

    // if we are turning left or right, then instantiate a "Turn" verb phrase
    if(direction != Directions::STRAIGHT) {
      auto turnIns = std::make_shared<VerbPhrase>("turn");
      turnIns->setStartRegion(thisRegionName);
      turnIns->setEndRegion(nextRegionName);
      turnIns->addDirection(direction);

      // see if there's a nearby landmark to include in the turn instruction
      auto pairIt = regionToPosesMap.find(thisRegion);
      auto posesInThisRegion = pairIt->second;
      geometry_msgs::PoseStamped boundary = posesInThisRegion.back();

      MapItem closestLandmark = getClosestLandmarkTo(boundary);
      closestLandmark.setCommonName(labelToCommonNameMap[closestLandmark.getName()]);
      //ROS_INFO("MapItem label: %s, common name: %s", closestLandmark.getName().c_str(), labelToCommonNameMap[closestLandmark.getName()].c_str());
      double landmarkToBoundaryDistance = closestLandmark.distanceTo(boundary.pose).data;

      // PRINT CODE
      ROS_INFO("Region: %s, closest landmark to boundary: %s with distance %lf",
                thisRegion.c_str(),
                closestLandmark.getName().c_str(),
                landmarkToBoundaryDistance);

      // if the landmark is close enough to the turn location, use it
      if (landmarkToBoundaryDistance < MapInfo::DISTANCE_THRESHOLD) {
        turnIns->addPreposition(Preposition("at", closestLandmark));
      }
      //If outside the "at" range, check if past/before
      else {
        auto startToMapItemDistance = closestLandmark.distanceTo(posesInThisRegion.front().pose).data;
        auto startToEndDistance = distanceBetween(posesInThisRegion.front().pose, boundary.pose).data;
        auto difference = startToEndDistance - startToMapItemDistance;
        if(std::abs(difference) < 6) {
          if(difference > 0) {
            turnIns->addPreposition(Preposition("past", closestLandmark));
          }
          else {
            turnIns->addPreposition(Preposition("before", closestLandmark));
          }
        }
      }
      // add the finished turn instruction to the instruction list
      instructionList.push_back(turnIns);
    }
  }

  auto arrival = std::make_shared<Arrival>(labelToCommonNameMap[regionList[regionList.size()-1]]);
  arrival->addDirection(getFinalDirection(regionList[regionList.size()-1]));
  instructionList.push_back(arrival);
}


// public method to generate natural language directions from
// the previously generated MapInfo information
std::string MapInfo::generateDirections(){
  // iterate over generated instructions, building natural language directions
  for(auto instr : instructionList) {
    std::string directionCommand = instr->toNaturalLanguage();
    directions.append(directionCommand);
  }
  return directions;
}



/* HELPER METHODS */

// input: any point on the map
// returns the MapItem with closest Euclidean distance to the specified point
MapItem MapInfo::getClosestLandmarkTo(geometry_msgs::PoseStamped pose) {
  double shortestDistance = std::numeric_limits<double>::max();
  MapItem* closestLandmark;
  // iterate over all landmarks
  for(auto& landmark : landmarkList) {
    auto dist = landmark.distanceTo(pose.pose).data;
    if(dist < shortestDistance) {
      closestLandmark = &landmark;
      shortestDistance = dist;
    }
  }
  return *closestLandmark;
}


// input: any point on the map
// returns the specified point's enclosing region
std::string MapInfo::getRegion(geometry_msgs::Pose currentLocation) {
  float robot_x = currentLocation.position.x;
  float robot_y = currentLocation.position.y;

  bwi_mapper::Point2f mapPoint(robot_x, robot_y);
  return translator.getLocationString(translator.getLocationIdx(mapPoint));
}

// input: A region name
// returns the Direction enum representing the angular difference between the two regions
Directions MapInfo::getFinalDirection(std::string finalRegion) {
  auto poseList = regionToPosesMap.find(finalRegion)->second;

  auto x1 = poseList.back().pose.position.x - poseList[0].pose.position.x;
  auto y1 = poseList[10].pose.position.y - poseList[0].pose.position.y;

  auto x2 = poseList.back().pose.position.x - poseList[0].pose.position.x;
  auto y2 = poseList.back().pose.position.y - poseList[0].pose.position.y;

  Eigen::Vector2d firstVector(x1, y1);
  Eigen::Vector2d secondVector(x2, y2);

  auto dot = firstVector.dot(secondVector);
  auto det = firstVector(0)*secondVector(1) - firstVector(1)*secondVector(0);
  auto angle = std::atan2(det, dot);

  //ROS_INFO("Angle between %s and %s: %lf", fromRegion.c_str(), toRegion.c_str(), angle);

  if(angle > MapInfo::ANGLE_THRESHOLD) {
    return Directions::LEFT;
  }
  else if(angle < -MapInfo::ANGLE_THRESHOLD) {
    return Directions::RIGHT;
  }

  return Directions::STRAIGHT;
}

// input: two region names
// returns the Direction enum representing the angular difference between the two regions
Directions MapInfo::getDirectionBetween(std::string fromRegion, std::string toRegion) {
  auto fromVector = regionToOrientationMap.find(fromRegion)->second;
  auto toVector = regionToOrientationMap.find(toRegion)->second;

  auto dot = fromVector.dot(toVector);
  auto det = fromVector(0)*toVector(1) - fromVector(1)*toVector(0);
  auto angle = std::atan2(det, dot);

  //ROS_INFO("Angle between %s and %s: %lf", fromRegion.c_str(), toRegion.c_str(), angle);

  if(angle > MapInfo::ANGLE_THRESHOLD) {
    return Directions::LEFT;
  }
  else if(angle < -MapInfo::ANGLE_THRESHOLD) {
    return Directions::RIGHT;
  }

  return Directions::STRAIGHT;
}



std_msgs::Float64 MapInfo::distanceBetween(geometry_msgs::Pose firstPose, geometry_msgs::Pose lastPose) {
  auto dx = firstPose.position.x - lastPose.position.x;
  auto dy = firstPose.position.y- lastPose.position.y;
  std_msgs::Float64 distance;
  distance.data = std::sqrt(std::pow(dx, 2) + std::pow(dy, 2));
  return distance;
}

bool MapInfo::readCommonNamesFile(const std::string& filename) {
  ROS_INFO(boost::filesystem::current_path().string().c_str());
  if (!boost::filesystem::exists(filename)) {
    return false;
  }

  std::ifstream fin(filename.c_str());

  YAML::Node doc;
//#ifdef HAVE_NEW_YAMLCPP
  doc = YAML::Load(fin);
// #else
//   YAML::Parser parser(fin);
//   parser.GetNextDocument(doc);
// #endif

  for (std::size_t i = 0; i < doc.size(); i++) {
    std::string label;
    std::string common_name;
    //doc[i]["name"] >> label;
    label = doc[i]["name"].as<std::string>();
    //doc[i]["common_name"] >> common_name;
    common_name = doc[i]["common_name"].as<std::string>();
    labelToCommonNameMap.emplace(std::make_pair(label, common_name));
  }

  fin.close();

  return true;
}
