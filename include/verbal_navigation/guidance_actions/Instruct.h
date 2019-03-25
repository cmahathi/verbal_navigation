#ifndef INSTRUCT_H
#define INSTRUCT_H

#include "verbal_navigation/guidance_actions/GuidanceAction.h"
#include "verbal_navigation/Wavenet.h"

class Instruct : public GuidanceAction {
    public:
    //TODO Add speech API client reference to constructor
    Instruct(std::vector<Region> regions, ros::ServiceClient& speechClient);
    void perform() override;
};

#endif