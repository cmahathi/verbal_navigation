#ifndef ACTION_DATA_H
#define ACTION_DATA_H

#include "verbal_navigation/guidance_actions/GuidanceActionTypes.h"
#include <geometry_msgs/Pose.h>

struct ActionData {
    ActionData(GuidanceActionTypes type, geometry_msgs::Pose startPose, geometry_msgs::Pose endPose, std::string instructionString);
    GuidanceActionTypes actionType;
    geometry_msgs::Pose initial_pose;
    geometry_msgs::Pose end_pose;
    size_t instructionSize;
    char* instructions;
    void debug();
};

#endif