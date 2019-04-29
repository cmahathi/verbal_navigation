#ifndef GUIDANCE_ACTION_H
#define GUIDANCE_ACTION_H

#include "verbal_navigation/Region.h"
#include <ros/ros.h>
#include "verbal_navigation/Robot_Action.h"
#include "verbal_navigation/guidance_actions/ActionData.h"
#include "verbal_navigation/guidance_actions/GuidanceActionTypes.h"


class GuidanceAction {
protected:
    std::vector<Region> regions;
    ros::ServiceClient& actionServiceClient;
    GuidanceAction(std::vector<Region> regions, ros::ServiceClient& serviceClient) : regions(regions), actionServiceClient(serviceClient) {};

public:
    virtual ActionData getActionData() = 0;
    virtual verbal_navigation::Robot_Action createMessage() = 0;
};

#endif