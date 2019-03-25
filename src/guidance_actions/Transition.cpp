#include "verbal_navigation/guidance_actions/Transition.h"

Transition::Transition(Region region, ros::ServiceClient& speechClient) : Instruct({region}, speechClient) {}

void Transition::perform() {
    for(auto& region : regions) {
        ROS_INFO("%s: Transition", region.getCommonName().c_str());
    }
}