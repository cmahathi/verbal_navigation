#ifndef INSTRUCT_H
#define INSTRUCT_H

#include "verbal_navigation/guidance_actions/GuidanceAction.h"

class Instruct : public GuidanceAction {
    public:
        Instruct(std::vector<Region> regions, GuidanceActionTypes t);
        void perform() override;
        std::vector<Region> getRegions();
        GuidanceActionTypes getType();


};

#endif