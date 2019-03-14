#include "verbal_navigation/guidance_actions/Instruct.h"

Instruct::Instruct(std::vector<Region> regions, GuidanceActionTypes t) : GuidanceAction(regions, t) {}

void Instruct::perform() {
    ROS_INFO("Instruct");
    for (int i = 0; i < regions.size(); i++) {
        ROS_INFO("\t%s", regions.at(i).getName().c_str());
    }

    
}