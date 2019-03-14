#ifndef GUIDANCE_ACTION_H
#define GUIDANCE_ACTION_H

#include "verbal_navigation/Region.h"

enum GuidanceActionTypes { LEAD = 'L', INSTRUCT = 'I', TRANSITION = 'T'};

class GuidanceAction {
protected:
    GuidanceAction(std::vector<Region> regions, GuidanceActionTypes t) : regions(regions), type(t) {};

public:
    std::vector<Region> regions;
    GuidanceActionTypes type;
    std::vector<Region> getRegions();
    GuidanceActionTypes getType();
    virtual void perform() = 0;
};

#endif