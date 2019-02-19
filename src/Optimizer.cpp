#include "verbal_navigation/Optimizer.h"

Optimizer::Optimizer (std::vector<Region> path, MapInfo f2, MapInfo f3) : regionPath(path), floor2(f2), floor3(f3) {
    ROS_INFO("Calculating optimal combination...");
    preprocess();
    ROS_INFO("Preprocessing finished");
}

void Optimizer::preprocess () {
    for (int i = 0; i < regionPath.size(); i++) {
        if (regionPath[i].getFloor() == 2) {
            ROS_INFO("Floor: %d, using mapinfo2", regionPath[i].getFloor());
            getLengthOfRegion(regionPath[i], floor2);
        }
        else {
            ROS_INFO("Floor: %d, using mapinfo3", regionPath[i].getFloor());
            getLengthOfRegion(regionPath[i], floor3);
        }
    }


}

void Optimizer::optimize () {

}

double Optimizer::getLengthOfRegion(Region r, MapInfo mapinfo) {
    std::string region_name = r.getName();
    ROS_INFO("Calculating length of region %s", region_name.c_str());
    auto regionToPoseMap = mapinfo.getRegionToPosesMap();
    ROS_INFO("Map size: %d", regionToPoseMap.size());
    // SEGFAULTING HERE CONNOR PLZ FIX IT
    auto pairIt = regionToPoseMap.find(region_name);
    auto poses = pairIt->second;
    double total = 0.0;
    ROS_INFO("HERE");
    for (int i = 0; i < poses.size()-1; i++) {
        ROS_INFO("In for loop, i = %d, poses size = %d", i, poses.size());
        total += distanceBetween(poses[i].pose, poses[i+1].pose);
    }

    return total;
}

double Optimizer::distanceBetween(geometry_msgs::Pose firstPose, geometry_msgs::Pose lastPose) {
    ROS_INFO("Getting distance between some poses");
    auto dx = firstPose.position.x - lastPose.position.x;
    auto dy = firstPose.position.y- lastPose.position.y;
    return std::sqrt(std::pow(dx, 2) + std::pow(dy, 2));
}