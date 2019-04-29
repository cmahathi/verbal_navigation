#include "verbal_navigation/guidance_actions/ActionData.h"
#include <ros/ros.h>

ActionData::ActionData(GuidanceActionTypes type, geometry_msgs::Pose startPose, geometry_msgs::Pose endPose, std::string instructionString) : actionType(type), initial_pose(startPose), end_pose(endPose), instructionSize(instructionString.size() + 1) {
    instructions = new char[instructionString.size()+1];
    instructionString.copy(instructions, instructionString.size());
    instructions[instructionString.size()] = '\0';
}

void ActionData::debug() {
    ROS_INFO("Action: %c, %d, %d, %d, %s", actionType, &initial_pose, &end_pose, instructionSize, instructions);
}