#ifndef INSTRUCT_H
#define INSTRUCT_H

#include "verbal_navigation/guidance_actions/GuidanceAction.h"

class Instruct : public GuidanceAction {
    public:
    //TODO Add speech API client reference to constructor
    Instruct(std::vector<Region> regions);
    void perform() override;
};

#endif