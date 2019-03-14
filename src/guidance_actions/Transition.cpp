#include "verbal_navigation/guidance_actions/Transition.h"

Transition::Transition(std::vector<Region> regions, GuidanceActionTypes t) : GuidanceAction(regions, t) {}

void Transition::perform() {
    ROS_INFO("Transition");
    for (int i = 0; i < regions.size(); i++) {
        ROS_INFO("\t%s", regions.at(i).getName().c_str());
    }
}