#include "verbal_navigation/guidance_actions/Transition.h"

Transition::Transition(Region region, ros::ServiceClient& speechClient) : Instruct({region}, speechClient) {}

ActionData Transition::getActionData() {
    for(auto& region : regions) {
        ROS_INFO("%s: Transition", region.getCommonName().c_str());
    }
}

verbal_navigation::Robot_Action Transition::createMessage() {
    verbal_navigation::Robot_Action msg;
    msg.action_type = 'T';
    msg.initial_pose = regions.front().getInitialPose();
    msg.end_pose = regions.back().getEndPose();
    msg.instructions = regions.front().getInstruction()->toNaturalLanguage();
    return msg;
}