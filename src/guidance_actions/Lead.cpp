#include "verbal_navigation/guidance_actions/Lead.h"

Lead::Lead(std::vector<Region> regions) : GuidanceAction(regions) {}

void Lead::perform() {
    for(auto& region : regions) {
        ROS_INFO("%s: Lead", region.getCommonName().c_str());
    }
}