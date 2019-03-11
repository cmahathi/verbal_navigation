#include "verbal_navigation/guidance_actions/Lead.h"

Lead::Lead(Region region) : GuidanceAction(region) {}

void Lead::perform() {
    ROS_INFO("%s: Lead", region.getCommonName().c_str());
}