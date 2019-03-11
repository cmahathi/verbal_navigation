#ifndef INSTRUCT_H
#define INSTRUCT_H

#include "verbal_navigation/guidance_actions/GuidanceAction.h"

class Instruct : public GuidanceAction {
    public:
    Instruct(Region region);
    void perform() override;
};

#endif