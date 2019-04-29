#ifndef LEAD_H
#define LEAD_H

#include "verbal_navigation/guidance_actions/GuidanceAction.h"

class Lead : public GuidanceAction {
    public:
    //TODO Add goToLocation client refernce to constructor
    Lead(std::vector<Region> regions, ros::ServiceClient& goToLocationClient);
    ActionData getActionData() override;
    verbal_navigation::Robot_Action createMessage() override;    
};

#endif