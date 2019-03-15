#ifndef GUIDANCE_ACTION_H
#define GUIDANCE_ACTION_H

#include "verbal_navigation/Region.h"
#include <ros/ros.h>

enum GuidanceActionTypes { LEAD = 'L', INSTRUCT = 'I', TRANSITION = 'T'};

class GuidanceAction {
protected:
    std::vector<Region> regions;
    std::shared_ptr<ros::ServiceClient> actionServiceClient;
    GuidanceAction(std::vector<Region> regions, std::shared_ptr<ros::ServiceClient> serviceClient) : regions(regions), actionServiceClient(serviceClient) {};

public:
    virtual void perform() = 0;
};

#endif