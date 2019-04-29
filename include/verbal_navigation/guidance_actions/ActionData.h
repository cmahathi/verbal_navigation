#ifndef ACTION_DATA_H
#define ACTION_DATA_H

#include "verbal_navigation/Actions.h"
#include <geometry_msgs/Pose.h>

struct ActionData {
    ActionData();
    // GuidanceActionTypes actionType;
    geometry_msgs::Pose initial_pose;
    geometry_msgs::Pose end_pose;
    char* instructions;
};

#endif