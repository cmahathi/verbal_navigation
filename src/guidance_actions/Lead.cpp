#include "verbal_navigation/guidance_actions/Lead.h"

Lead::Lead(std::vector<Region> regions, GuidanceActionTypes t) : GuidanceAction(regions, t) {}

void Lead::perform() {
    ROS_INFO("Lead");
    for (int i = 0; i < regions.size(); i++) {
        ROS_INFO("\t%s", regions.at(i).getName().c_str());
    }
}