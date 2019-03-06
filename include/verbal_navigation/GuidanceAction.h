#ifndef GUIDANCE_ACTION_H
#define GUIDANCE_ACTION_H

#include "verbal_navigation/Region.h"

enum GuidanceActions { LEAD = 'L', INSTRUCT = 'I', TRANSITION = 'T'};

class GuidanceAction {
    Region region;

    public:
        GuidanceAction(Region region);
        virtual void perform() = 0;
};

#endif