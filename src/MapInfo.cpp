#include "verbal_navigation/MapInfo.h"


//Constructor
MapInfo::MapInfo(bwi_logical_translator::BwiLogicalTranslator& trans, std::vector<geometry_msgs::PoseStamped> path)
  : translator(trans), poseList(path), regionList(), regionToPosesMap() {

  if(poseList.empty()) {
    ROS_INFO("ERROR: Initialized with empty path");
  }
  buildRegionAndPointsInfo();
  buildRegionOrientationInfo();
  buildRegionsToLandmarksMap();
  buildInstructions();
}



void MapInfo::buildRegionAndPointsInfo() {
  //adds the first region the path goes through
  regionList.push_back(getRegion(poseList.front().pose));
  //ROS_INFO("Added region: %s\n", getRegion(translator, poseList.front()).c_str());

  // add each point's region to the region list, and add the point to the regionToPosesMap
  for(size_t i = 0; i < poseList.size(); ++i) {
    auto currentLocation = poseList[i];
    //ROS_INFO("Orientation: %lf, %lf, %lf, %lf", currentLocation.pose.orientation.x,currentLocation.pose.orientation.y,currentLocation.pose.orientation.z,currentLocation.pose.orientation.w);

    std::string region = getRegion(currentLocation.pose);

    //only adds the region to the list of regions if the previous pose was
    //not in the region
    if (region.compare("") != 0) {
      if( (regionList.back()).compare(region) != 0) {
        regionList.push_back(region);
        // ROS_INFO("Added region: %s\n", region.c_str());
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
}


void MapInfo::buildRegionOrientationInfo() {
  // calculate representative orientation for every region
  std::map<std::string, std::vector<geometry_msgs::PoseStamped>>::iterator it;
  for ( it = regionToPosesMap.begin(); it != regionToPosesMap.end(); it++ ) {

    std::vector<geometry_msgs::PoseStamped> posesInRegionList = it->second;
    auto x = posesInRegionList.front().pose.position.x - posesInRegionList.back().pose.position.x;
    auto y = posesInRegionList.front().pose.position.y - posesInRegionList.back().pose.position.y;
    Eigen::Vector2d regionPathVector(x, y);

    regionToOrientationMap.emplace(std::make_pair(it->first, regionPathVector));
    // ROS_INFO("r2o %s %lf %lf", regionToOrientationMap.find(it->first)->first.c_str(), regionPathVector(0), regionPathVector(1));
  }
}

void MapInfo::buildRegionsToLandmarksMap() {

  const auto& landmarkNameToPositionMap = translator.getObjectApproachMap();
  for (auto const& pair : landmarkNameToPositionMap) {
    Landmark landmark(pair.first, pair.second);
    landmarkList.push_back(landmark);

    //ROS_INFO("object found: %s", pose.first.c_str());
    // geometry_msgs::Pose pose = pair.second;
    //ROS_INFO("Location: %lf %lf %lf", pose.position.x, pose.position.y, pose.position.z);
    std::string region = getRegion(landmark.getPose());

    auto regionToLandmarksPair = regionToLandmarksMap.find(region);

    // if it doesn't exist already
    if(regionToLandmarksPair == regionToLandmarksMap.end()) {
      std::vector<Landmark> newList;
      newList.push_back(landmark);
      regionToLandmarksMap.emplace(std::make_pair(region, newList));
    } else {
      // if the region already exists in this map, just add this landmark to the region's value
      regionToLandmarksPair->second.push_back(landmark);
    }
  }
  // CODE TO PRINT
  std::map<std::string, std::vector<std::string>>::iterator it;
  for (auto it = regionToLandmarksMap.begin(); it != regionToLandmarksMap.end(); it++ ) {
    ROS_INFO("Region Name: %s", it->first.c_str());
    for (auto landmark : it->second) {
      ROS_INFO("  Landmark: %s", landmark.getName().c_str());
    }
  }
}

void MapInfo::buildInstructions() {

  // iterate through all the regions except the last one
  for (int ix = 0; ix < regionList.size() - 1; ix ++) {
    std::string thisRegion = regionList[ix];
    std::string nextRegion = regionList[ix + 1];

    auto direction = shouldTurnBetween(thisRegion, nextRegion);

    VerbPhrase travelIns = VerbPhrase("travel");
    travelIns.setStartRegion(thisRegion);
    travelIns.setEndRegion(thisRegion);
    instructionList.push_back(travelIns);

    // if we are turning left or right, the instantiate a "Turn" verb phrase
    if(direction != Directions::STRAIGHT) {

      VerbPhrase turnIns = VerbPhrase("turn");
      turnIns.setStartRegion(thisRegion);
      turnIns.setEndRegion(nextRegion);
      turnIns.addDirection(direction);


      // see if there's a close landmark to include in the turn instruction
      auto pairIt = regionToPosesMap.find(thisRegion);
      auto posesInThisRegion = pairIt->second;
      geometry_msgs::PoseStamped boundry = posesInThisRegion.back();

      Landmark closestLandmark = getClosestLandmarkTo(boundry);
      double closestDistance = closestLandmark.distanceTo(boundry.pose).data;

      // PRINT CODE
      ROS_INFO("Region: %s, closest landmark to boundry: %s with distance %lf",
                thisRegion.c_str(),
                closestLandmark.getName().c_str(),
                closestDistance);

      // if the landmark is close enough to the turn location, use it
      if (closestDistance < MapInfo::DISTANCE_THRESHOLD) {
        Preposition turnPrep = Preposition("at", closestLandmark);
        turnIns.addPreposition(turnPrep);
      }


      // add the finished turn instruction to the instruction list
      instructionList.push_back(turnIns);
    }
  }

  // iterate through the newly created instructions
  // and convert to natural language
  for(VerbPhrase instr : instructionList) {
    ROS_INFO(instr.toNaturalLanguage().c_str());
  }

  //Add arrival predicate
}




/* HELPER METHODS */

Landmark MapInfo::getClosestLandmarkTo(geometry_msgs::PoseStamped pose) {
  double shortestDistance = std::numeric_limits<double>::max();
  Landmark* closestLandmark;
  for(auto& landmark : landmarkList) {
    auto dist = landmark.distanceTo(pose.pose).data;
    if(dist < shortestDistance) {
      closestLandmark = &landmark;
      shortestDistance = dist;
    }
  }

  return *closestLandmark;
}


// takes in a location on the map and returns the region it is in
std::string MapInfo::getRegion(geometry_msgs::Pose currentLocation) {
  float robot_x = currentLocation.position.x;
  float robot_y = currentLocation.position.y;

  bwi_mapper::Point2f mapPoint(robot_x, robot_y);
  return translator.getLocationString(translator.getLocationIdx(mapPoint));
}


// returns a direction to turn between two input regions
Directions MapInfo::shouldTurnBetween(std::string fromRegion, std::string toRegion) {
  auto fromVector = regionToOrientationMap.find(fromRegion)->second;
  auto toVector = regionToOrientationMap.find(toRegion)->second;

  auto dot = fromVector.dot(toVector);
  auto det = fromVector(0)*toVector(1) - fromVector(1)*toVector(0);
  auto angle = std::atan2(det, dot);

  ROS_INFO("Angle betwenn %s and %s: %lf", fromRegion.c_str(), toRegion.c_str(), angle);

  if(angle < -MapInfo::ANGLE_THRESHOLD) {
    return Directions::RIGHT;
  }
  else if(angle > MapInfo::ANGLE_THRESHOLD) {
    return Directions::LEFT;
  }

  return Directions::STRAIGHT;
}

/* GETTER METHODS */
//
// std::map<std::string, std::vector<geometry_msgs::PoseStamped>> MapInfo::getregionToPosesMap(){
//   return regionToPosesMap;
// }
//
// std::vector<std::string> MapInfo::getRegionList(){
//   return regionList;
// }
