#ifndef TRANSITION_H
#define TRANSITION_H

#include "verbal_navigation/guidance_actions/GuidanceAction.h"

//Should extend instruct?
class Transition : public GuidanceAction {
    public:
    //TODO add speech client reference to constructor
    Transition(Region region);
    void perform() override;
};

#endif