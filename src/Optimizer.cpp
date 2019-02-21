#include "verbal_navigation/Optimizer.h"

Optimizer::Optimizer (std::vector<Region> path, MapInfo f2, MapInfo f3) : regionPath(path), floor2(f2), floor3(f3) {
    ROS_INFO("Calculating optimal combination...");
    preprocess();
    ROS_INFO("Preprocessing finished");
    //printPathInfo();
}

void Optimizer::optimize () {

}

void Optimizer::preprocess () {
    // for (int i = 0; i < regionPath.size(); i++) {
    //   ROS_INFO("%d: %s", i, regionPath[i].getName().c_str());
    // }
    for (int i = 0; i < regionPath.size(); i++) {
        if (regionPath[i].getFloor() == 2) {
            regionPath[i].setLength(getLengthOfRegion(regionPath[i], floor2));
            if (i > 0 && regionPath[i-1].getFloor() == 2) {
                regionPath[i].has_door = isDoorBetweenRegions(regionPath[i-1], regionPath[i], floor2);
            }
            else {
                regionPath[i].has_door = false;
            }
        }
        else {
            regionPath[i].setLength(getLengthOfRegion(regionPath[i], floor3));
            if (i > 0 && regionPath[i-1].getFloor() == 3) {
                regionPath[i].has_door = isDoorBetweenRegions(regionPath[i-1], regionPath[i], floor3);
            }
            else {
                regionPath[i].has_door = false;
            }
        }
        regionPath[i].traversibility = calculateTraversibility(regionPath[i]);
    }
    calculateRobotTimes();
    calculateBaseHumanTimes();
}

void Optimizer::calculateRobotTimes() {
    for (int i = 0; i < regionPath.size(); i++) {
        regionPath[i].robot_time = regionPath[i].length / ROBOT_VELOCITY * regionPath[i].traversibility;
    }
}

void Optimizer::calculateBaseHumanTimes() {
    for (int i = 0; i < regionPath.size(); i++) {
        regionPath[i].base_human_time = regionPath[i].length / HUMAN_VELOCITY * regionPath[i].num_neighbors;
    }
}

void Optimizer::printPathInfo() {
    ROS_INFO("Printing Path info... Size: %d", regionPath.size());
    for (int i = 0; i < regionPath.size(); i++) {
        // ROS_INFO("Region: %s\n\tLength: %f\n\tType: %s\n\tDoor: %d\n\tTraversibility: %f\n\tNeighbors: %d\n",
        //         regionPath[i].getName().c_str(), regionPath[i].getLength(), regionPath[i].getType(), regionPath[i].getDoor(),
        //         regionPath[i].getTraversibility(), regionPath[i].getNumNeighbors());
        ROS_INFO("Region: %s\n\tLength: %f\n\tType: \n\tDoor: %d\n\tTraversibility: %f\n\tNeighbors: %d\n\tRobot Time: %f\n\tHuman Time (base): %f",
                regionPath[i].getName().c_str(), regionPath[i].getLength(), regionPath[i].getDoor(),
                regionPath[i].getTraversibility(), regionPath[i].getNumNeighbors(), regionPath[i].robot_time,
                regionPath[i].base_human_time);
    }
}

double Optimizer::calculateTraversibility (Region r) {
    int traversibility = 0;
    switch (r.getType()) {
        case RegionType::ROOM:  traversibility += 1;
                                break;
        case RegionType::HALLWAY:  traversibility += 1;
                                break;
        case RegionType::OPEN_SPACE:  traversibility += 3;
                                break;
        case RegionType::ELEVATOR:  traversibility += 10;
                                break;
    }
    if (r.getDoor())
        traversibility += 5;
    return traversibility;
}

bool Optimizer::isDoorBetweenRegions(Region a, Region b, MapInfo mapinfo) {
    std::vector<bwi_planning_common::Door> doors = mapinfo.translator.getDoorMap();
    std::string name1 = a.getName();
    std::string name2 = b.getName();
    for (int i = 0; i < doors.size(); i++) {
        if ((doors[i].approach_names[0] == name1 && doors[i].approach_names[1] == name2) || (doors[i].approach_names[1] == name1 && doors[i].approach_names[0] == name2)) {
            return true;
        }
    }
    return false;
}

double Optimizer::getLengthOfRegion(Region r, MapInfo mapinfo) {
    std::string region_name = r.getName();
    auto regionToPoseMap = mapinfo.getRegionToPosesMap();

    auto poses = regionToPoseMap[region_name];
    double total = 0.0;
    for (int i = 0; i < poses.size()-1; i++) {
        total += distanceBetween(poses[i].pose, poses[i+1].pose);
    }
    return total;
}

double Optimizer::distanceBetween(geometry_msgs::Pose firstPose, geometry_msgs::Pose lastPose) {
    auto dx = firstPose.position.x - lastPose.position.x;
    auto dy = firstPose.position.y- lastPose.position.y;
    return std::sqrt(std::pow(dx, 2) + std::pow(dy, 2));
}
