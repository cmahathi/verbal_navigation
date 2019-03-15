#ifndef GUIDANCE_ACTION_H
#define GUIDANCE_ACTION_H

#include "verbal_navigation/Region.h"

enum GuidanceActionTypes { LEAD = 'L', INSTRUCT = 'I', TRANSITION = 'T'};

class GuidanceAction {
protected:
    std::vector<Region> regions;
    GuidanceAction(std::vector<Region> regions) : regions(regions) {};

public:
    virtual void perform() = 0;
};

#endif