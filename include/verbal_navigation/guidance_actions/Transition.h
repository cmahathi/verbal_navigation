#ifndef TRANSITION_H
#define TRANSITION_H

#include "verbal_navigation/guidance_actions/GuidanceAction.h"

class Transition : public GuidanceAction {
    public:
        Transition(std::vector<Region> regions, GuidanceActionTypes t);
        std::vector<Region> getRegions();
        GuidanceActionTypes getType();
        void perform() override;
};

#endif