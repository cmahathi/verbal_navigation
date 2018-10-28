#include "verbal_navigation/MapInfo.h"

// constructor
MapInfo::MapInfo(bwi_logical_translator::BwiLogicalTranslator& trans, std::vector<geometry_msgs::PoseStamped> path)
  : translator(trans), poseList(path) {

  if(poseList.empty()) {
    ROS_INFO("ERROR: Initialized with empty path");
  }
  buildRegionAndPointsInfo();
  buildRegionOrientationInfo();
  buildRegionsToLandmarksMap();
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
        ROS_INFO("REGION: %s\n", region.c_str());
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
    auto x = posesInRegionList.front().pose.position.x - posesInRegionList.back().pose.position.x;
    auto y = posesInRegionList.front().pose.position.y - posesInRegionList.back().pose.position.y;
    Eigen::Vector2d regionPathVector(x, y);

    regionToOrientationMap.emplace(std::make_pair(it->first, regionPathVector));
    // ROS_INFO("r2o %s %lf %lf", regionToOrientationMap.find(it->first)->first.c_str(), regionPathVector(0), regionPathVector(1));
  }
}


// builds a "Landmark" object for each landmark; builds a list of landmarks;
// build a map of regions to landmarks in that region
void MapInfo::buildRegionsToLandmarksMap() {

  // get a map of landmark names to landmark positions from the translator
  const auto& landmarkNameToPositionMap = translator.getObjectApproachMap();
  for (auto const& pair : landmarkNameToPositionMap) {
    Landmark landmark(pair.first, pair.second);
    landmarkList.push_back(landmark);

    std::string region = getRegion(landmark.getPose());

    auto regionToLandmarksPair = regionToLandmarksMap.find(region);

    // if region doesn't have any landmarks yet, add this first one
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


// builds a list of predicates representing the verbal instructions for this path
void MapInfo::buildInstructions() {

  // iterate through all the regions except the last one
  for (int ix = 0; ix < regionList.size() - 1; ix ++) {
    std::string thisRegion = regionList[ix];
    std::string nextRegion = regionList[ix + 1];

    auto direction = getDirectionBetween(thisRegion, nextRegion);

    auto travelIns = std::make_shared<VerbPhrase>("travel");
    travelIns->setStartRegion(thisRegion);
    travelIns->setEndRegion(thisRegion);
    instructionList.push_back(travelIns);

    // if we are turning left or right, then instantiate a "Turn" verb phrase
    if(direction != Directions::STRAIGHT) {
      auto turnIns = std::make_shared<VerbPhrase>("turn");
      turnIns->setStartRegion(thisRegion);
      turnIns->setEndRegion(nextRegion);
      turnIns->addDirection(direction);

      // see if there's a nearby landmark to include in the turn instruction
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

  

  //Arrival arrival = Arrival(regionList[regionList.size()-1]);
  //arrival.addDirection(Directions::STRAIGHT);
  //instructionList.push_back(&arrival);

  for(int i = 0; i < instructionList.size(); i++) {
    ROS_INFO("%s", instructionList[i]->toNaturalLanguage().c_str());

  //Add arrival predicate
}


// public method to generate natural language directions from
// the previously generated information
// void MapInfo::generateDirections(){
//   // iterate over generated instructions, building natural language directions
//   for(VerbPhrase instr : instructionList) {
//     std::string directionCommand = instr.toNaturalLanguage();\
//     directions.append(directionCommand);
//   }
//   ROS_INFO(directions.c_str());
// }



/* HELPER METHODS */

// input: any point on the map
// returns the Landmark with closest Euclidean distance to the specified point
Landmark MapInfo::getClosestLandmarkTo(geometry_msgs::PoseStamped pose) {
  double shortestDistance = std::numeric_limits<double>::max();
  Landmark* closestLandmark;
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


// input: two region names
// returns the Direction enum representing the angular difference between the two regions
Directions MapInfo::getDirectionBetween(std::string fromRegion, std::string toRegion) {
  auto fromVector = regionToOrientationMap.find(fromRegion)->second;
  auto toVector = regionToOrientationMap.find(toRegion)->second;

  auto dot = fromVector.dot(toVector);
  auto det = fromVector(0)*toVector(1) - fromVector(1)*toVector(0);
  auto angle = std::atan2(det, dot);

  ROS_INFO("Angle between %s and %s: %lf", fromRegion.c_str(), toRegion.c_str(), angle);

  if(angle < -MapInfo::ANGLE_THRESHOLD) {
    return Directions::RIGHT;
  }
  else if(angle > MapInfo::ANGLE_THRESHOLD) {
    return Directions::LEFT;
  }

  return Directions::STRAIGHT;
}
