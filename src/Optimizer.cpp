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
    debug = true;
    currentMinTime = 100000.0;
    if(debug) {
        ROS_INFO("Regions in path: %d", segmentedPath.size());
        ROS_INFO("-----------------RECURSION TREE------------------------");
    }
    calculateRegionTime(0.0, 0, 0, 'L', false, " ");
    ROS_INFO("Final Path: %s\nTime: %lf", pathToString(currentMinPath).c_str(), currentMinTime);

    if (debug) {
        ROS_INFO("\n\n\n");
        ROS_INFO("-----------------RECURSION TREE------------------------");
    }
    calculateRegionTime(0.0, 0, 0, 'I', false, " ");
    ROS_INFO("Final Path: %s\nTime: %lf", pathToString(currentMinPath).c_str(), currentMinTime);

}

void Optimizer::calculateRegionTime(double accumulatedTime, int numInstructedRegions, int regionCounter, char action, bool transition, std::string spaces) {
    updatePath(action);
    if (debug) {
        ROS_INFO("%s%c", spaces.c_str(), action);
    }

    accumulatedTime = calculateAccumulatedTime(accumulatedTime, numInstructedRegions, regionCounter, action);
    //ROS_INFO("AccumulatedTime: %lf, minTime %lf", accumulatedTime, currentMinTime);
    if (accumulatedTime > currentMinTime){
        backtrackPath();
        return;
    }
    if (regionCounter == segmentedPath.size()) {
        updateMin(accumulatedTime);
        backtrackPath();

        return;
    }
    regionCounter++;
    if (domainTransition(regionCounter) || transition) {
        calculateRegionTime(accumulatedTime, numInstructedRegions+1, regionCounter, 'T', false, spaces + "  ");
        calculateRegionTime(accumulatedTime, numInstructedRegions+1, regionCounter, 'I', true, spaces + "  ");
    }
    else {
        if (action == 'I') {
            calculateRegionTime(accumulatedTime, numInstructedRegions+1, regionCounter, 'I', transition, spaces + "  ");
        }
        else if (action == 'T') {
            calculateRegionTime(accumulatedTime, 0, regionCounter, 'I', false, spaces + "  ");
            calculateRegionTime(accumulatedTime, 0, regionCounter, 'L', false, spaces + "  ");
        }
        else {
            // action = 'L'
            calculateRegionTime(accumulatedTime, numInstructedRegions+1, regionCounter, 'I', transition, spaces + "  ");
            calculateRegionTime(accumulatedTime, 0, regionCounter, 'L', transition, spaces + "  ");
        }
    }
    backtrackPath();

}

void Optimizer::updatePath(char action) {
    currentPath.push_back(action);
}

void Optimizer::backtrackPath() {
    currentPath.pop_back();
}

double Optimizer::calculateAccumulatedTime(double accumulatedTime, int numInstructedRegions, int regionCounter, char action) {
    if (action == 'I' || action == 'T') {
        double acc = accumulatedTime + segmentedPath[regionCounter].base_human_time * (double)(numInstructedRegions+1);
        if (numInstructedRegions == 0) {
            acc += SPEECH_TIME;
        }
        //ROS_INFO("Accumulated Time (in calculation) from T or F: %lf",acc);
        return acc;
    }
    else {
        return accumulatedTime + segmentedPath[regionCounter].robot_time;
    }
}

void Optimizer::updateMin(double accumulatedTime) {
    currentMinPath = currentPath;
    currentMinTime = accumulatedTime;
}

bool Optimizer::domainTransition(int regionCount) {
    if (regionCount >= segmentedPath.size() - 2)
        return false;
    return domains.isRegionTransition(segmentedPath[regionCount].getName(), segmentedPath[regionCount+1].getName());
}


void Optimizer::preprocess () {
    // for (int i = 0; i < segmentedPath.size(); i++) {
    //   ROS_INFO("%d: %s %f", i, segmentedPath[i].getName().c_str(), segmentedPath[i].getLength());
    // }
    for (int i = 0; i < segmentedPath.size(); i++) {
        segmentedPath[i].setTraversibility(calculateTraversibility(segmentedPath[i]));
    }
    // ROS_INFO("Calculating robot times");
    calculateRobotTimes();
    // ROS_INFO("Calculating human times");
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
    // ROS_INFO("Printing Path info... Size: %d", segmentedPath.size());
    // for (int i = 0; i < segmentedPath.size(); i++) {
    //     // ROS_INFO("Region: %s\n\tLength: %f\n\tType: %s\n\tDoor: %d\n\tTraversibility: %f\n\tNeighbors: %d\n",
    //     //         segmentedPath[i].getName().c_str(), segmentedPath[i].getLength(), segmentedPath[i].getType(), segmentedPath[i].getDoor(),
    //     //         segmentedPath[i].getTraversibility(), segmentedPath[i].getNumNeighbors());
    //     //ROS_INFO("Region: %s\n\tLength: %f\n\tType: \n\tDoor: %d\n\tTraversibility: %f\n\tNeighbors: %d\n\tRobot Time: %f\n\tHuman Time (base): %f",
    //             segmentedPath[i].getName().c_str(), segmentedPath[i].getLength(), segmentedPath[i].getDoor(),
    //             segmentedPath[i].getTraversibility(), segmentedPath[i].getNumNeighbors(), segmentedPath[i].robot_time,
    //             segmentedPath[i].base_human_time);
    // }
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

std::string Optimizer::pathToString (std::vector<char> path) {
    std::string result = "";
    //ROS_INFO("IN PATH TO STRING METHOD");
    for (int i = 0; i < path.size(); i++) {
        result.push_back(path[i]);
    }
    return result;
}
