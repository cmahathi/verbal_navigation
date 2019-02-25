#include "verbal_navigation/Optimizer.h"

// TODO it seems weird that this is all hard coded for floors 2 and 3. We always have at most 2 floors in our path: an upper floor and a lower floor.
// (No one needs to use a third floor as a transition floor unless they go from one basement to another, an edge case we don't care to cover right now)
Optimizer::Optimizer (RegionPath& regionPath, MapInfo f2, MapInfo f3) : segmentedPath(regionPath.path), floor2(f2), floor3(f3) {
    ROS_INFO("Calculating optimal combination...");
    preprocess();
    ROS_INFO("Preprocessing finished");
    //printPathInfo();
}

void Optimizer::optimize () {

}

void Optimizer::preprocess () {
    // for (int i = 0; i < segmentedPath.size(); i++) {
    //   ROS_INFO("%d: %s %f", i, segmentedPath[i].getName().c_str(), segmentedPath[i].getLength());
    // }
    for (int i = 0; i < segmentedPath.size(); i++) {
        // if (segmentedPath[i].getFloor() == 2) {
        //     if (i > 0 && segmentedPath[i-1].getFloor() == 2) {
        //         segmentedPath[i].setDoor(floor2.isDoorBetweenRegions(segmentedPath[i-1], segmentedPath[i]));
        //     }
        //     else {
        //         segmentedPath[i].setDoor(false);
        //     }
        // }
        // else {
        //     if (i > 0 && segmentedPath[i-1].getFloor() == 3) {
        //         segmentedPath[i].setDoor(floor3.isDoorBetweenRegions(segmentedPath[i-1], segmentedPath[i]));
        //     }
        //     else {
        //         segmentedPath[i].setDoor(false);
        //     }
        // }
        segmentedPath[i].setTraversibility(calculateTraversibility(segmentedPath[i]));
    }
    ROS_INFO("Calculating robot times");
    calculateRobotTimes();
    ROS_INFO("Calculating human times");
    calculateBaseHumanTimes();
    // printPathInfo();
}

void Optimizer::calculateRobotTimes() {
    for (int i = 0; i < segmentedPath.size(); i++) {
        segmentedPath[i].robot_time = segmentedPath[i].getLength() / ROBOT_VELOCITY * segmentedPath[i].getTraversibility();
    }
}

void Optimizer::calculateBaseHumanTimes() {
    for (int i = 0; i < segmentedPath.size(); i++) {
        segmentedPath[i].base_human_time = segmentedPath[i].getLength() / HUMAN_VELOCITY * segmentedPath[i].getNumNeighbors();
    }
}

void Optimizer::printPathInfo() {
    ROS_INFO("Printing Path info... Size: %d", segmentedPath.size());
    for (int i = 0; i < segmentedPath.size(); i++) {
        // ROS_INFO("Region: %s\n\tLength: %f\n\tType: %s\n\tDoor: %d\n\tTraversibility: %f\n\tNeighbors: %d\n",
        //         segmentedPath[i].getName().c_str(), segmentedPath[i].getLength(), segmentedPath[i].getType(), segmentedPath[i].getDoor(),
        //         segmentedPath[i].getTraversibility(), segmentedPath[i].getNumNeighbors());
        ROS_INFO("Region: %s\n\tLength: %f\n\tType: \n\tDoor: %d\n\tTraversibility: %f\n\tNeighbors: %d\n\tRobot Time: %f\n\tHuman Time (base): %f",
                segmentedPath[i].getName().c_str(), segmentedPath[i].getLength(), segmentedPath[i].getDoor(),
                segmentedPath[i].getTraversibility(), segmentedPath[i].getNumNeighbors(), segmentedPath[i].robot_time,
                segmentedPath[i].base_human_time);
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

double Optimizer::distanceBetween(geometry_msgs::Pose firstPose, geometry_msgs::Pose lastPose) {
    auto dx = firstPose.position.x - lastPose.position.x;
    auto dy = firstPose.position.y- lastPose.position.y;
    return std::sqrt(std::pow(dx, 2) + std::pow(dy, 2));
}
