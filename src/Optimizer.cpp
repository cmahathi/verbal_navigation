#include "verbal_navigation/Optimizer.h"

// TODO it seems weird that this is all hard coded for floors 2 and 3. We always have at most 2 floors in our path: an upper floor and a lower floor.
// (No one needs to use a third floor as a transition floor unless they go from one basement to another, an edge case we don't care to cover right now)
Optimizer::Optimizer (RegionPath& regionPath, MapInfo f2, MapInfo f3) : segmentedPath(regionPath.path), floor2(f2), floor3(f3), domains(), currentMinTime(0), currentPath(), currentMinPath(), debug(false) {
    ROS_INFO("Calculating optimal combination...");
    preprocess();
    ROS_INFO("Preprocessing finished");
    //printPathInfo();
}

void Optimizer::optimize () {
    debug = false;
    currentMinTime = std::numeric_limits<double>::max();
    if(debug) {
        ROS_INFO("Regions in path: %d", segmentedPath.size());
        ROS_INFO("-----------------RECURSION TREE------------------------");
    }
    calculateRegionTime(0.0, 0, 0, 'L', false);
    ROS_INFO("Final Path: %s\nTime: %lf", pathToString(currentMinPath).c_str(), currentMinTime);

    if (debug) {
        ROS_INFO("\n\n\n");
        ROS_INFO("-----------------RECURSION TREE------------------------");
    }
    calculateRegionTime(0.0, 0, 0, 'I', false);
    ROS_INFO("Final Path: %s\nTime: %lf", pathToString(currentMinPath).c_str(), currentMinTime);
}

void Optimizer::calculateRegionTime(double accumulatedTime, int numInstructedRegions, int regionCounter, char action, bool transition) {
    updatePath(action);
    accumulatedTime = calculateAccumulatedTime(accumulatedTime, numInstructedRegions, regionCounter, action);

    if (debug) {
        ROS_INFO("Region %d: %c", regionCounter, action);
        ROS_INFO("AccumulatedTime: %lf, minTime %lf", accumulatedTime, currentMinTime);
    }

    if (accumulatedTime > currentMinTime) {
        backtrackPath();
        return;
    }
    if (regionCounter == segmentedPath.size() - 1) {
        updateMin(accumulatedTime);
        backtrackPath();
        return;
    }
    regionCounter++;
    if (transition || domainTransition(regionCounter)) {
        calculateRegionTime(accumulatedTime, numInstructedRegions+1, regionCounter, 'T', false);
        calculateRegionTime(accumulatedTime, numInstructedRegions+1, regionCounter, 'I', true);
    }
    else {
        if (action == 'I') {
            calculateRegionTime(accumulatedTime, numInstructedRegions+1, regionCounter, 'I', transition);
        }
        else if (action == 'T') {
            calculateRegionTime(accumulatedTime, 0, regionCounter, 'I', false);
            calculateRegionTime(accumulatedTime, 0, regionCounter, 'L', false);
        }
        else {
            // action == 'L'
            calculateRegionTime(accumulatedTime, numInstructedRegions+1, regionCounter, 'I', transition);
            calculateRegionTime(accumulatedTime, 0, regionCounter, 'L', transition);
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
        double acc = accumulatedTime + segmentedPath.at(regionCounter).base_human_time * (double)(numInstructedRegions+1);
        if (numInstructedRegions == 0) {
            acc += SPEECH_TIME;
        }
        return acc;
    }
    else {
        return accumulatedTime + segmentedPath.at(regionCounter).robot_time;
    }
}

void Optimizer::updateMin(double accumulatedTime) {
    currentMinPath = currentPath;
    currentMinTime = accumulatedTime;
}

bool Optimizer::domainTransition(int regionCount) {
    if (regionCount >= segmentedPath.size() - 1)
        return false;
    return domains.isDomainTransition(segmentedPath.at(regionCount).getName(), segmentedPath.at(regionCount+1).getName());
}


void Optimizer::preprocess () {
    for (int i = 0; i < segmentedPath.size(); i++) {
        segmentedPath.at(i).setTraversibility(calculateTraversibility(segmentedPath.at(i)));
    }
    ROS_INFO("Calculating robot times");
    calculateRobotTimes();
    ROS_INFO("Calculating human times");
    calculateBaseHumanTimes();
    // printPathInfo();
}

void Optimizer::calculateRobotTimes() {
    for (auto& region : segmentedPath) {
        region.robot_time = region.getLength() / ROBOT_VELOCITY * region.getTraversibility();
    }
}

void Optimizer::calculateBaseHumanTimes() {
    for (auto& region : segmentedPath) {
        region.base_human_time = region.getLength() / HUMAN_VELOCITY * region.getNumNeighbors();
    }
}

void Optimizer::printPathInfo() {
    ROS_INFO("Printing Path info... Size: %d", segmentedPath.size());
    for (int i = 0; i < segmentedPath.size(); i++) {
        // ROS_INFO("Region: %s\n\tLength: %f\n\tType: %s\n\tDoor: %d\n\tTraversibility: %f\n\tNeighbors: %d\n",
        //         segmentedPath[i].getName().c_str(), segmentedPath[i].getLength(), segmentedPath[i].getType(), segmentedPath[i].getDoor(),
        //         segmentedPath[i].getTraversibility(), segmentedPath[i].getNumNeighbors());
        // ROS_INFO("Region: %s\n\tLength: %f\n\tType: \n\tDoor: %d\n\tTraversibility: %f\n\tNeighbors: %d\n\tRobot Time: %f\n\tHuman Time (base): %f",
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

std::string Optimizer::pathToString (std::vector<char> path) {
    std::string result = "";
    for (int i = 0; i < path.size(); i++) {
        result.push_back(path[i]);
    }
    return result;
}
