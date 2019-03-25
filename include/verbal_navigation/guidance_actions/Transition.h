#ifndef TRANSITION_H
#define TRANSITION_H

#include "verbal_navigation/guidance_actions/Instruct.h"

//Should extend instruct
class Transition : public Instruct {
    public:
    //TODO add speech client reference to constructor
    Transition(Region region, ros::ServiceClient& speechClient);
    void perform() override;
};

#endif