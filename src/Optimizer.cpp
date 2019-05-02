#include "verbal_navigation/Optimizer.h"

// TODO it seems weird that this is all hard coded for floors 2 and 3. We always have at most 2 floors in our path: an upper floor and a lower floor.
// (No one needs to use a third floor as a transition floor unless they go from one basement to another, an edge case we don't care to cover right now)
Optimizer::Optimizer (RegionPath& regionPath, MapInfo f2, MapInfo f3) : segmentedPath(regionPath.path), floor2(f2), floor3(f3), domains(), currentMinTime(0), currentPath(), currentMinPath(), debug(false), optimized(false) {
    ROS_INFO("Calculating optimal combination...");
    preprocess();
    ROS_INFO("Preprocessing finished");
    //printPathInfo();
}

void Optimizer::optimize () {
    debug = false;
    currentMinTime = std::numeric_limits<double>::max();
    std::string robot_id = domains.getRobotByRegion(segmentedPath.at(0).getName());
    if(debug) {
        ROS_INFO("Regions in path: %d", segmentedPath.size());
        ROS_INFO("-----------------RECURSION TREE------------------------");
    }
    calculateRegionTime(0.0, 0, 0, GuidanceActionTypes::LEAD, false, robot_id);
    //ROS_INFO("Final Path: %s\nTime: %lf", pathToString(currentMinPath).c_str(), currentMinTime);

    if (debug) {
        ROS_INFO("\n\n\n");
        ROS_INFO("-----------------RECURSION TREE------------------------");
    }
    calculateRegionTime(0.0, 0, 0, GuidanceActionTypes::INSTRUCT, false, robot_id);
    ROS_INFO("Final Path: %s\nTime: %lf", pathToString(currentMinPath).c_str(), currentMinTime);
}

void Optimizer::calculateRegionTime(double accumulatedTime, int numInstructedRegions, int regionCounter, GuidanceActionTypes action, bool transition, std::string robot_id) {
    // Base case: we have calcualated the time for every region in the path
    if (regionCounter == segmentedPath.size()) {
        // Result: we have found a new minimum time combination
        updateMin(accumulatedTime);
        return;
    }

    // Try appending this action to our current path permutation
    updatePath(action);
    accumulatedTime = calculateAccumulatedTime(accumulatedTime, numInstructedRegions, regionCounter, action);

    if (debug) {
        ROS_INFO("Region %d: %c", regionCounter, action);
        ROS_INFO("AccumulatedTime: %lf, minTime %lf", accumulatedTime, currentMinTime);
    }

    // If this makes the path take longer than our current minumum, abandon this branch
    if (accumulatedTime > currentMinTime) {
        backtrackPath();
        return;
    }

    regionCounter++;
    // Try the remaining possible follow-up actions to this action for the next region
    if (transition || domainTransition(regionCounter, robot_id)) {
        calculateRegionTime(accumulatedTime, numInstructedRegions+1, regionCounter, GuidanceActionTypes::TRANSITION, false, domains.getRobotByRegion(segmentedPath.at(regionCounter+1).getName()));
        calculateRegionTime(accumulatedTime, numInstructedRegions+1, regionCounter, GuidanceActionTypes::INSTRUCT, true, robot_id);
    }
    else {
        if (action == GuidanceActionTypes::INSTRUCT) {
            calculateRegionTime(accumulatedTime, numInstructedRegions+1, regionCounter, GuidanceActionTypes::INSTRUCT, transition, robot_id);
        }
        else if (action == GuidanceActionTypes::TRANSITION) {
            // ROS_INFO("Transition to %s at %s", robot_id.c_str(), segmentedPath.at(regionCounter-1).getName().c_str());
            calculateRegionTime(accumulatedTime, 0, regionCounter, GuidanceActionTypes::INSTRUCT, false, robot_id);
            calculateRegionTime(accumulatedTime, 0, regionCounter, GuidanceActionTypes::LEAD, false, robot_id);
        }
        else {
            // action == GuidanceActionTypes::LEAD
            calculateRegionTime(accumulatedTime, numInstructedRegions+1, regionCounter, GuidanceActionTypes::INSTRUCT, transition, robot_id);
            calculateRegionTime(accumulatedTime, 0, regionCounter, GuidanceActionTypes::LEAD, transition, robot_id);
        }
    }
    backtrackPath();
}

void Optimizer::updatePath(GuidanceActionTypes action) {
    currentPath.push_back(action);
}

void Optimizer::backtrackPath() {
    currentPath.pop_back();
}

double Optimizer::calculateAccumulatedTime(double accumulatedTime, int numInstructedRegions, int regionCounter, GuidanceActionTypes action) {
    if (action == GuidanceActionTypes::INSTRUCT) {
        double acc = accumulatedTime + segmentedPath.at(regionCounter).base_human_time * (double)(numInstructedRegions+1);
        acc += SPEECH_TIME;
        
        return acc;
    }
    else if (action == GuidanceActionTypes::TRANSITION) {
        return accumulatedTime + (segmentedPath.at(regionCounter).base_human_time)/(segmentedPath.at(regionCounter).getNumNeighbors() * .5) + SPEECH_TIME;
    }
    else {
        return accumulatedTime + segmentedPath.at(regionCounter).robot_time;
    }
}

void Optimizer::updateMin(double accumulatedTime) {
    currentMinPath = currentPath;
    currentMinTime = accumulatedTime;
}

bool Optimizer::domainTransition(int regionCount, std::string cur_robot_id) {
    if (regionCount >= segmentedPath.size() - 1)
        return false;

    std::string next_robot_id = domains.getRobotByRegion(segmentedPath.at(regionCount+1).getName());
    if (next_robot_id.compare("") != 0 && next_robot_id.compare(cur_robot_id) != 0) {
        ROS_INFO("Domain transition detected: %s to %s, robots %s to %s", segmentedPath.at(regionCount).getName().c_str(), segmentedPath.at(regionCount+1).getName().c_str(), cur_robot_id.c_str(), next_robot_id.c_str());
        return true;
    }
    ROS_INFO("NOT domain transition: %s to %s, robots %s to %s", segmentedPath.at(regionCount).getName().c_str(), segmentedPath.at(regionCount+1).getName().c_str(), cur_robot_id.c_str(), next_robot_id.c_str());
    return false;
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
        case RegionType::OPEN_SPACE:  traversibility += 5;
                                break;
        case RegionType::ELEVATOR:  traversibility += 10;
                                break;
    }
    if (r.getDoor())
        traversibility += 5;
    return traversibility;
}

std::string Optimizer::pathToString (std::vector<GuidanceActionTypes> path) {
    std::string result = "";
    for (int i = 0; i < path.size(); i++) {
        result.push_back(path[i]);
    }
    return result;
}

std::vector<GuidanceActionTypes> Optimizer::getOptimalGuidanceSequence() {
    if(!optimized) {
        optimize();
    }

    return currentMinPath;
}
