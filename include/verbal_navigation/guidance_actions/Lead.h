#ifndef LEAD_H
#define LEAD_H

#include "verbal_navigation/guidance_actions/GuidanceAction.h"

class Lead : public GuidanceAction {
    public:
        Lead(std::vector<Region> regions, GuidanceActionTypes t);
        std::vector<Region> getRegions();
        GuidanceActionTypes getType();
        void perform() override;
};

#endif