#ifndef GUIDANCE_ACTION_H
#define GUIDANCE_ACTION_H

#include "verbal_navigation/Region.h"
#include <ros/ros.h>
#include "verbal_navigation/Robot_Action.h"
#include "verbal_navigation/DomainInfo.h"

enum GuidanceActionTypes { LEAD = 'L', INSTRUCT = 'I', TRANSITION = 'T'};

class GuidanceAction {
protected:
    std::vector<Region> regions;
    DomainInfo domains;
    ros::ServiceClient& actionServiceClient;
    GuidanceAction(std::vector<Region> regions, ros::ServiceClient& serviceClient) : regions(regions), actionServiceClient(serviceClient), domains() {};

public:
    virtual void perform() = 0;
    virtual verbal_navigation::Robot_Action createMessage() = 0;
};

#endif