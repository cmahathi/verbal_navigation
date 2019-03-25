#ifndef SEQUENCER_H
#define SEQUENCER_H

#include "verbal_navigation/Actions.h"
#include <queue>
#include <memory>

class Sequencer {
    std::queue<std::shared_ptr<GuidanceAction>> guidanceActionSequence;

    void generateGuidanceActionSequence(std::vector<GuidanceActionTypes>& actionSequence, std::vector<Region>& regionSequence);

public:
    Sequencer(std::vector<GuidanceActionTypes>& actionSequence, std::vector<Region>& regionSequence, ros::NodeHandle& nh);
    std::queue<std::shared_ptr<GuidanceAction>> getGuidanceActionSequence();
};

#endif