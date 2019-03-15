#include "verbal_navigation/guidance_actions/Instruct.h"

Instruct::Instruct(std::vector<Region> regions) : GuidanceAction(regions) {}

void Instruct::perform() {
    for(auto& region : regions) {
        if(region.getInstruction() == nullptr) {
            ROS_ERROR("Instruction not set for %s", region.getCommonName().c_str());
        }
        else {
            ROS_INFO("%s: %s", region.getCommonName().c_str(), region.getInstruction()->toNaturalLanguage().c_str());
        }
    }
}