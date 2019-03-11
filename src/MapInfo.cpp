#include "verbal_navigation/MapInfo.h"
#include <boost/filesystem.hpp>
#include <fstream>
#include <libgen.h>
#include <stdexcept>
#include <stdio.h>
#include <yaml-cpp/yaml.h>

// constructor
MapInfo::MapInfo(bwi_logical_translator::BwiLogicalTranslator& trans, std::vector<geometry_msgs::PoseStamped> path, std::string dest, std::string floor_id)
  : translator(trans), poseList(path), destinationCommonName(dest) {

    // TODO parse the first char as an int instead?
  if (floor_id[0] == '2')
    floor = 2;
  else
    floor = 3;

  if(poseList.empty()) {
    ROS_ERROR("Generating plan failed: pose list is empty. Quitting.");
    return;
  }

  // I think that, rather than doing this for all regions before we load our path, we should load the attributes only for the regions in our RegionPath object after buildRegionsAndPointsInfo is called.
  regions.floor_id = floor_id;
  buildRegionAndPointsInfo();
  readAttributesFile(boost::filesystem::current_path().string() + "/src/multimap/" + floor_id + "/region_attributes.yaml");
  ROS_INFO("Building Region Orientation Info...");
  buildRegionOrientationInfo();
  ROS_INFO("Building Regions To Items Map...");
  buildRegionsToMapItemsMap();
  ROS_INFO("Building Instructions...");
  //buildInstructions();
  ROS_INFO("MapInfo Done!");
}


// turns path of poses into list of regions, and builds map of regions to poses
void MapInfo::buildRegionAndPointsInfo() {
  // ROS_INFO("START REGIONS");
  // add the robot's initial region to the RegionPath
  Region firstRegion(getRegion(poseList.front().pose));
  regions.path.push_back(firstRegion);
  // ROS_INFO("INITIAL REGION: %s", firstRegion.getName().c_str());
  //ROS_INFO("Added region: %s\n", getRegion(translator, poseList.front()).c_str());

  // add each point's region to the region list, and add that point to the Region
  for(size_t i = 0; i < poseList.size(); ++i) {
    auto currentLocation = poseList[i];
    // ROS_INFO("Orientation: %lf, %lf, %lf, %lf", currentLocation.pose.orientation.x,currentLocation.pose.orientation.y,currentLocation.pose.orientation.z,currentLocation.pose.orientation.w);

    std::string currentRegionName = getRegion(currentLocation.pose);

    // only add region to RegionPath if it's not same as last seen region
    if (!currentRegionName.empty()) {
      // If we are in a new region, add it to the RegionPath
      if( (regions.path.back().getName()).compare(currentRegionName) != 0) {
        auto currentRegion = Region(currentRegionName);
        if(isDoorBetweenRegions(currentRegion, regions.path.back())) {
          currentRegion.setDoor(true);
        }
        regions.path.push_back(currentRegion);
        // ROS_INFO("REGION: %s", currentRegionName.c_str());        
       }
      auto& currentRegion = regions.path.back();
      if(currentRegion.getPath().size() > 1) {
        currentRegion.setLength(currentRegion.getLength() + distanceBetween(currentLocation.pose, currentRegion.getPath().back().pose).data);
      }
      currentRegion.appendPoseToPath(currentLocation);
    }
  }
  // ROS_INFO("END REGIONS");
}


// determines representative orientation for each region using first and last pose in that region
void MapInfo::buildRegionOrientationInfo() {
  // calculate representative orientation for every region
  for (auto& region : regions.path) {

    std::vector<geometry_msgs::PoseStamped> posesInRegionPath = region.getPath();
    auto x = posesInRegionPath.back().pose.position.x - posesInRegionPath.front().pose.position.x;
    auto y = posesInRegionPath.back().pose.position.y - posesInRegionPath.front().pose.position.y;
    Eigen::Vector2d regionPathVector(x, y);

    regionToOrientationMap.emplace(std::make_pair(region.getName(), regionPathVector));
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
std::string MapInfo::buildInstructions(std::vector<Region>& regions, bool robotTransition, bool elevator, int nextFloor = 0) {

  // iterate through all the regions except the last one
  // ROS_INFO("Num Regions: %d", regions.path.size());
  // ROS_INFO("Regions in path: %d", regions.size());
  // for (int i = 0; i < regions.size(); i++) {
  //   ROS_INFO("Region: %s", regions[i].getCommonName().c_str());
  // }

  for (int ix = 0; ix < regions.size() - 2; ix ++) {
    auto& thisRegion = regions[ix];
    
    auto& nextRegion = regions[ix + 1];
    auto thisRegionName = thisRegion.getCommonName();
    auto nextRegionName = nextRegion.getCommonName();

    auto travelIns = std::make_shared<VerbPhrase>("go");
    travelIns->setStartRegion(thisRegionName);
    travelIns->setEndRegion(thisRegionName);
    MapItem regionItem(thisRegion.getName());
    regionItem.setCommonName(labelToCommonNameMap[regionItem.getName()]);
    travelIns->addPreposition(Preposition("through", regionItem));

    thisRegion.setInstruction(travelIns);

    // if we are turning left or right, then instantiate a "Turn" verb phrase
    auto direction = getDirectionBetween(thisRegion, nextRegion);
    if(direction != Directions::STRAIGHT) {
      auto turnIns = std::make_shared<VerbPhrase>("turn");
      turnIns->setStartRegion(thisRegionName);
      turnIns->setEndRegion(nextRegionName);
      turnIns->addDirection(direction);

      // see if there's a nearby landmark to include in the turn instruction
      auto posesInThisRegion = thisRegion.getPath();
      geometry_msgs::PoseStamped boundary = posesInThisRegion.back();

      MapItem closestLandmark = getClosestLandmarkTo(boundary);
      // TODO: Can we build the landmark MapItems as we parse the YAML file?
      closestLandmark.setCommonName(labelToCommonNameMap[closestLandmark.getName()]);
      //ROS_INFO("MapItem label: %s, common name: %s", closestLandmark.getName().c_str(), labelToCommonNameMap[closestLandmark.getName()].c_str());
      double landmarkToBoundaryDistance = closestLandmark.distanceTo(boundary.pose).data;

      // PRINT CODE
      // ROS_INFO("Region: %s, closest landmark to boundary: %s with distance %lf",
      //           thisRegion.getName().c_str(),
      //           closestLandmark.getName().c_str(),
      //           landmarkToBoundaryDistance);

      if (mapItemInRegion(thisRegion.getName(), closestLandmark) || mapItemInRegion(nextRegion.getName(), closestLandmark)) {

        // if the landmark is close enough to the turn location, use it
        if (landmarkToBoundaryDistance < MapInfo::DISTANCE_THRESHOLD) {
          turnIns->addPreposition(Preposition("at", closestLandmark));
        }
        // If outside the "at" range, check if past/before
        else {
          auto startToMapItemDistance = closestLandmark.distanceTo(posesInThisRegion.front().pose).data;
          auto startToEndDistance = distanceBetween(posesInThisRegion.front().pose, boundary.pose).data;
          auto difference = startToEndDistance - startToMapItemDistance;
          if(std::abs(difference) < 1) {
            if(difference > 0) {
              turnIns->addPreposition(Preposition("past", closestLandmark));
            }
            else {
              turnIns->addPreposition(Preposition("before", closestLandmark));
            }
          }
          else {
            // we're not turning at, past, or before a landmark. So just turn "into the next regoin".
            // ROS_ERROR(nextRegionName.c_str());
            MapItem nextRegion(nextRegionName);
            nextRegion.setCommonName(nextRegionName);
            turnIns->addPreposition(Preposition("towards", nextRegion));
          }
        }
      }
      // add the finished turn instruction to its respective region
      thisRegion.setInstruction(turnIns);

      //ROS_INFO("Instruction generated: %s", turnIns->toNaturalLanguage().c_str());
    } // end of the if clause for turning
  }

  // generate the final predicate to tell the user how to arrive at destination
  std::string regionName = labelToCommonNameMap[regions.at(regions.size()-1).getName()];
  auto arrival = std::make_shared<Arrival>(regionName, robotTransition, elevator, nextFloor);
  //auto arrival = std::make_shared<Arrival>(destinationCommonName);  
  arrival->addDirection(getDirectionBetween(regions.at(regions.size()-2), (regions.at(regions.size()-1))));
  regions.back().setInstruction(arrival);

  return generateDirections(regions);
}

std::string MapInfo::generateDirections(std::vector<Region>& regionList) {
  // iterate over generated instructions, building natural language directions and ignoring duplicate instructions.
  std::string lastInstruction = "";
  for(auto& region : regionList) {
    std::string naturalInstruction = region.getInstruction()->toNaturalLanguage();
    if(lastInstruction != naturalInstruction) {
      directions.append(naturalInstruction);
    }
    lastInstruction = naturalInstruction;
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
Directions MapInfo::getFinalDirection(Region finalRegion) {
  auto poseList = finalRegion.getPath();;

  auto x1 = poseList[10].pose.position.x - poseList[0].pose.position.x;
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
Directions MapInfo::getDirectionBetween(Region fromRegion, Region toRegion) {
  auto fromVector = regionToOrientationMap.find(fromRegion.getName())->second;
  auto toVector = regionToOrientationMap.find(toRegion.getName())->second;

  auto dot = fromVector.dot(toVector);
  auto det = fromVector(0)*toVector(1) - fromVector(1)*toVector(0);
  auto angle = std::atan2(det, dot);

  //ROS_INFO("Angle between %s and %s: %lf", fromRegion.getName().c_str(), toRegion..getName().c_str(), angle);

  if(angle > MapInfo::ANGLE_THRESHOLD) {
    return Directions::LEFT;
  }
  else if(angle < -MapInfo::ANGLE_THRESHOLD) {
    return Directions::RIGHT;
  }

  return Directions::STRAIGHT;
}

// This method might belong in the actual translator itself (using regnion names as args), if we can get permission to move it there.
bool MapInfo::isDoorBetweenRegions(Region a, Region b) {
    std::vector<bwi_planning_common::Door> doors = translator.getDoorList();
    std::string nameA = a.getName();
    std::string nameB = b.getName();
    for (int i = 0; i < doors.size(); i++) {
        if ((doors[i].approach_names[0] == nameA && doors[i].approach_names[1] == nameB) || (doors[i].approach_names[1] == nameA && doors[i].approach_names[0] == nameB)) {
            return true;
        }
    }
    return false;
}



std_msgs::Float64 MapInfo::distanceBetween(geometry_msgs::Pose firstPose, geometry_msgs::Pose lastPose) {
  auto dx = firstPose.position.x - lastPose.position.x;
  auto dy = firstPose.position.y- lastPose.position.y;
  std_msgs::Float64 distance;
  distance.data = std::sqrt(std::pow(dx, 2) + std::pow(dy, 2));
  return distance;
}

bool MapInfo::mapItemInRegion(std::string region, MapItem item) {
  std::string name = item.getName();
  auto itemList = regionToMapItemsMap[region];
  for (MapItem i : itemList) {
    if (name == i.getName()) {
      return true;
    }
  }
  return false;
}

bool MapInfo::readAttributesFile(const std::string& filename) {
  ROS_INFO(boost::filesystem::current_path().string().c_str());

  if (!boost::filesystem::exists(filename)) {
    ROS_INFO("COMMON NAME FILE NOT FOUND\nRun from verbal_nav directory");
    return false;
  }

  std::ifstream fin(filename.c_str());

  YAML::Node doc;
  doc = YAML::Load(fin);
  const YAML::Node region_node = doc["regions"];

  // Populate all the regions in our path with annotation data
  for (std::size_t i = 0; i < region_node.size(); i++){
    std::string label = region_node[i]["name"].as<std::string>();
    std::string common_name = region_node[i]["common_name"].as<std::string>();
    labelToCommonNameMap.emplace(std::make_pair(label, common_name));

    Region* region = regions.getRegion(label); 
    
    if(region != NULL) {
      int region_type = region_node[i]["type"].as<int>();
      int neighbors = region_node[i]["neighbors"].as<int>();

      region->setCommonName(common_name);
      region->setType(region_type);
      region->setFloor(floor);
      region->setNumNeighbors(neighbors);
    }
  }

  const YAML::Node landmark_node = doc["landmarks"];
  for (std::size_t i = 0; i < landmark_node.size(); i++){
    std::string label = landmark_node[i]["name"].as<std::string>();
    std::string common_name = landmark_node[i]["common_name"].as<std::string>();
    labelToCommonNameMap.emplace(std::make_pair(label, common_name));
  }


  // for (std::size_t i = 0; i < doc.size(); i++) {
  //   std::string label;
  //   std::string common_name;
  //   //doc[i]["name"] >> label;
  //   label = doc[i]["name"].as<std::string>();
  //   //doc[i]["common_name"] >> common_name;
  //   common_name = doc[i]["common_name"].as<std::string>();
  //   labelToCommonNameMap.emplace(std::make_pair(label, common_name));
  //   bool has_door = doc[i]["has_door"].as<bool>();
  //   Region reg(label);
  //   reg.setCommonName(common_name);
  //   reg.setDoor(has_door);
  //   regions.push_back(reg);
  // }


  ROS_INFO("Region List Size: %d", regions.path.size());

  fin.close();

  return true;
}

RegionPath MapInfo::getRegionPath() {
  return regions;
}

std::map<std::string, std::vector<geometry_msgs::PoseStamped>> MapInfo::getRegionToPosesMap() {
  return regionToPosesMap;
}
