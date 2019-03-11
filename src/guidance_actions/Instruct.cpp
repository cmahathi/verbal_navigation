#include "verbal_navigation/guidance_actions/Instruct.h"

Instruct::Instruct(Region region) : GuidanceAction(region) {}

void Instruct::perform() {
    ROS_INFO("%s: Instruct", region.getCommonName().c_str());
}