#ifndef LEAD_H
#define LEAD_H

#include "verbal_navigation/guidance_actions/GuidanceAction.h"

class Lead : public GuidanceAction {
    public:
    Lead(Region region);
    void perform() override;
};

#endif