#include "verbal_navigation/guidance_actions/Transition.h"

Transition::Transition(Region region) : GuidanceAction({region}) {}

void Transition::perform() {
    for(auto& region : regions) {
        ROS_INFO("%s: Transition", region.getCommonName().c_str());
    }
}