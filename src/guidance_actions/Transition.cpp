#include "verbal_navigation/guidance_actions/Transition.h"

Transition::Transition(Region region, ros::ServiceClient& speechClient) : Instruct({region}, speechClient) {}

void Transition::perform() {
    for(auto& region : regions) {
        ROS_INFO("%s: Transition", region.getCommonName().c_str());
    }
}

verbal_navigation::Robot_Action Transition::createMessage() {
    verbal_navigation::Robot_Action msg;
    msg.action_type = "T";
    msg.initial_pose = regions.at(0).getInitialPose();
    msg.end_pose = regions.at(regions.size()-1).getEndPose();
    msg.instructions = regions.at(0).getInstruction()->toNaturalLanguage();
    return msg;
}