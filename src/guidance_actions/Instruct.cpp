#include "verbal_navigation/guidance_actions/Instruct.h"

Instruct::Instruct(std::vector<Region> regions, ros::ServiceClient& speechClient) : GuidanceAction(regions, speechClient)  {}

void Instruct::perform() {
    std::string directions;
    for(auto& region : regions) {
        if(region.getInstruction() == nullptr) {
            ROS_ERROR("Instruction not set for %s", region.getCommonName().c_str());
        }
        else {
            directions += region.getInstruction()->toNaturalLanguage();
            //ROS_INFO("%s: %s", region.getCommonName().c_str(), region.getInstruction()->toNaturalLanguage().c_str());
        }
    }

	verbal_navigation::Wavenet wavService;
    wavService.request.text = directions;
	actionServiceClient.call(wavService);
    ROS_INFO("Speaking %s", directions.c_str());
}

verbal_navigation::Robot_Action Instruct::createMessage() {
    verbal_navigation::Robot_Action msg;
    msg.action_type = "I";
    std::string directions = "";
    for(auto& region : regions) {
        if(region.getInstruction() == nullptr) {
            ROS_ERROR("Instruction not set for %s", region.getCommonName().c_str());
        }
        else {
            directions += region.getInstruction()->toNaturalLanguage();
            //ROS_INFO("%s: %s", region.getCommonName().c_str(), region.getInstruction()->toNaturalLanguage().c_str());
        }
    }
    msg.robot_id = domains.getRobotByRegion(regions.at(0).getName());
    msg.instructions = directions;
    msg.initial_pose = regions.at(0).getInitialPose();
    msg.end_pose = regions.at(regions.size()-1).getEndPose();
    return msg;
}
