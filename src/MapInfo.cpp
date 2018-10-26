#include "verbal_navigation/MapInfo.h"


//Constructor
MapInfo::MapInfo(bwi_logical_translator::BwiLogicalTranslator& trans, std::vector<geometry_msgs::PoseStamped> path)
  : translator(trans), poseList(path), regionList(), regionToPointsMap() {

  if(poseList.empty()) {
    ROS_INFO("ERROR: Initialized with empty path");
  }
  buildRegionAndPointsInfo();
  buildRegionOrientationInfo();
  buildRegionsToLandmarksMap();
  // buildInstructions();
}



void MapInfo::buildRegionAndPointsInfo() {
  //adds the first region the path goes through
  regionList.push_back(getRegion(poseList.front().pose));
  //ROS_INFO("Added region: %s\n", getRegion(translator, poseList.front()).c_str());

  // add each point's region to the region list, and add the point to the regionToPointsMap
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
      // add point to regionToPointsMap
      auto points = regionToPointsMap.find(region);
      if(points == regionToPointsMap.end()) {
        std::vector<geometry_msgs::PoseStamped> newList;
        newList.push_back(currentLocation);
        regionToPointsMap.emplace(std::make_pair(region, newList));
      } else {
        points->second.push_back(currentLocation);
      }
    }
  }
}


void MapInfo::buildRegionOrientationInfo() {

  // calculate representative orientation for every region
  std::map<std::string, std::vector<geometry_msgs::PoseStamped>>::iterator it;
  for ( it = regionToPointsMap.begin(); it != regionToPointsMap.end(); it++ ) {

    int numPoses = poseList.size();
    std::vector<geometry_msgs::PoseStamped> poseList = it->second;
    Eigen::MatrixXd pointMatrix(numPoses, 3); // this is "A" matrix in the best-fit equation
    Eigen::VectorXd zeroes = Eigen::VectorXd::Zero(numPoses, 1); // this is the "B" vector in the best-fit equation

    // populate the matrix with every point in this region
    for (int ix = 0; ix < numPoses; ix++) {
      geometry_msgs::Point thisPoint = poseList[ix].pose.position;
      pointMatrix(ix, 0) = thisPoint.x;
      pointMatrix(ix, 1) = thisPoint.y;
      pointMatrix(ix, 2) = 1;
    }

    // calculate the best-fit line
    Eigen::Vector3d best_fit = pointMatrix.bdcSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(zeroes);
    ROS_INFO("Region Name: %s", it->first.c_str());
    ROS_INFO("best fit is:");
    ROS_INFO("x: %lf", best_fit(0));
    ROS_INFO("y: %lf", best_fit(1));

    regionToOrientationMap.emplace(std::make_pair(it->first, best_fit));
    // ROS_INFO(best_fit);

  }

  ROS_INFO("all regions have been best-fitted.");
}

  // std::map<std::string, std::vector<geometry_msgs::PoseStamped>> map) {
  //   std::map<std::string, float> result;
  //   double yawTotal = 0.0;
  //   int numPoints = 0;
  //   for (int i = 0; i < regionList.size(); i++) {
  //     ROS_INFO("Getting new pose list");
  //     std::string region = regionList.at(i);
  //     if (regionToPointsMap.find(region) != regionToPointsMap.end()) {
  //       std::vector<geometry_msgs::PoseStamped> poseList = regionToPointsMap.find(region)->second;
  //       ROS_INFO("Region: %s", region.c_str());
  //       for (int i = 0; i < poseList.size(); i++) {
  //         geometry_msgs::PoseStamped pose = poseList.at(i);
  //         ROS_INFO("Position: %lf %lf %lf", pose.pose.position.x, pose.pose.position.y, pose.pose.position.z);
  //         ROS_INFO("Orietation: %lf %lf %lf %lf", pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z, pose.pose.orientation.w);
  //         tf::Quaternion q(
  //           pose.pose.orientation.x,
  //           pose.pose.orientation.y,
  //           pose.pose.orientation.z,
  //           pose.pose.orientation.w
  //         );
  //           tf::Matrix3x3 m(q);
  //        double roll, pitch, yaw;
  //
  //        m.getRPY(roll, pitch, yaw);
  //        ROS_INFO ("ROLL: %lf, PITCH: %lf, YAW: %lf", roll, pitch, yaw);
  //       }
  //   }



void MapInfo::buildRegionsToLandmarksMap() {

  landmarkNameToPositionMap = translator.getObjectApproachMap();
  for (auto const& pair : landmarkNameToPositionMap) {
    std::string landmark = pair.first;
    //ROS_INFO("object found: %s", object.c_str());
    geometry_msgs::Pose pose = pair.second;
    //ROS_INFO("Location: %lf %lf %lf", pose.position.x, pose.position.y, pose.position.z);
    std::string region = getRegion(pose);

    auto regionToLandmarksPair = regionToLandmarksMap.find(region);

    // if it doesn't exist already
    if(regionToLandmarksPair == regionToLandmarksMap.end()) {
      std::vector<std::string> newList;
      newList.push_back(landmark);
      regionToLandmarksMap.emplace(std::make_pair(region, newList));
    } else {
      // if the region already exists in this map, just add this landmark to the region's value
      regionToLandmarksPair->second.push_back(landmark);
    }
  }
  // CODE TO PRINT
  // std::map<std::string, std::vector<std::string>>::iterator it;
  // for ( it = regionToLandmarksMap.begin(); it != regionToLandmarksMap.end(); it++ ) {
  //   ROS_INFO("Region Name: %s", it->first.c_str());
  //   for (std::string landmark : it->second) {
  //     ROS_INFO("  Landmark: %s", landmark.c_str());
  //   }
  // }

}

void MapInfo::buildInstructions() {
  for (int ix = 0; ix < regionList.size(); ix ++) {
    std::string thisRegion = regionList[ix];
    std::string nextRegion = "";
    if (ix != regionList.size()) {
      nextRegion = regionList[ix + 1];
    }

    VerbPhrase travelIns = VerbPhrase("travel");
    travelIns.setStartRegion(thisRegion);
    travelIns.setEndRegion(thisRegion);

    VerbPhrase turnIns = VerbPhrase("turn");
    turnIns.setStartRegion(thisRegion);
    turnIns.setEndRegion(nextRegion);


  }

}

/* HELPER METHODS */

//takes in a location on the map and returns the region it is in
std::string MapInfo::getRegion(geometry_msgs::Pose currentLocation) {
  float robot_x = currentLocation.position.x;
  float robot_y = currentLocation.position.y;

  bwi_mapper::Point2f mapPoint(robot_x, robot_y);
  return translator.getLocationString(translator.getLocationIdx(mapPoint));
}

/* GETTER METHODS */
//
// std::map<std::string, std::vector<geometry_msgs::PoseStamped>> MapInfo::getRegionToPointsMap(){
//   return regionToPointsMap;
// }
//
// std::vector<std::string> MapInfo::getRegionList(){
//   return regionList;
// }
