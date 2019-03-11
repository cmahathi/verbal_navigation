#ifndef TRANSITION_H
#define TRANSITION_H

#include "verbal_navigation/guidance_actions/GuidanceAction.h"

class Transition : public GuidanceAction {
    public:
    Transition(Region region);
    void perform() override;
};

#endif