#include "verbal_navigation/guidance_actions/Lead.h"

Lead::Lead(std::vector<Region> regions, ros::ServiceClient& goToLocationClient) : GuidanceAction(regions, goToLocationClient) {}

ActionData Lead::getActionData() {
    return ActionData(GuidanceActionTypes::LEAD, regions.front().getInitialPose(), regions.back().getEndPose(), std::string());
}

// void Lead::debug() {
    // for(auto& region : regions) {
    //     ROS_INFO("%s: Lead", region.getCommonName().c_str());
    // }
// }
verbal_navigation::Robot_Action Lead::createMessage() {
    verbal_navigation::Robot_Action msg;
    msg.action_type = 'L';
    msg.initial_pose = regions.front().getInitialPose();
    msg.end_pose = regions.back().getEndPose();
    return msg;
}
