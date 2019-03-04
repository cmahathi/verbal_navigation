#ifndef SEQUENCER_H
#define SEQUENCER_H

#include "verbal_navigation/GuidanceAction.h"
#include <queue>
#include <memory>

class Sequencer {
    std::vector<GuidanceActions> actionSequence;
    std::vector<Region> regionSequence;

    std::queue<std::shared_ptr<GuidanceAction>> guidanceActionSequence;

public:
    Sequencer(std::vector<GuidanceActions>& actionSequence, std::vector<Region>& regionSequence);
};

#endif