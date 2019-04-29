#include "verbal_navigation/guidance_actions/Instruct.h"

Instruct::Instruct(std::vector<Region> regions, ros::ServiceClient& speechClient) : GuidanceAction(regions, speechClient)  {}

ActionData  Instruct::getActionData() {
    return ActionData(GuidanceActionTypes::INSTRUCT, regions.front().getInitialPose(), regions.back().getEndPose(), getCombinedDirections());
}

// void Instruct::debug(){
//     std::string directions;
//     for(auto& region : regions) {
//         if(region.getInstruction() == nullptr) {
//             ROS_ERROR("Instruction not set for %s", region.getCommonName().c_str());
//         }
//         else {
//             directions += region.getInstruction()->toNaturalLanguage();
//             //ROS_INFO("%s: %s", region.getCommonName().c_str(), region.getInstruction()->toNaturalLanguage().c_str());
//         }
//     }

// 	verbal_navigation::Wavenet wavService;
//     wavService.request.text = directions;
// 	actionServiceClient.call(wavService);
//     ROS_INFO("Speaking %s", directions.c_str());
// }

verbal_navigation::Robot_Action Instruct::createMessage() {
    verbal_navigation::Robot_Action msg;
    msg.action_type = GuidanceActionTypes::INSTRUCT;
    
    msg.instructions = getCombinedDirections();
    msg.initial_pose = regions.front().getInitialPose();
    msg.end_pose = regions.back().getEndPose();
    return msg;
}
