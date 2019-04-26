#include "verbal_navigation/guidance_actions/Lead.h"

Lead::Lead(std::vector<Region> regions, ros::ServiceClient& goToLocationClient) : GuidanceAction(regions, goToLocationClient) {}

void Lead::perform() {
    for(auto& region : regions) {
        ROS_INFO("%s: Lead", region.getCommonName().c_str());
    }
}

verbal_navigation::Robot_Action Lead::createMessage() {
    verbal_navigation::Robot_Action msg;
    msg.robot_id = domains.getRobotByRegion(regions.at(0).getName());
    msg.action_type = "L";
    msg.initial_pose = regions.at(0).getInitialPose();
    msg.end_pose = regions.at(regions.size()-1).getEndPose();
    return msg;
}
