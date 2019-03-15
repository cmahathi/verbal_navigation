#include "verbal_navigation/Sequencer.h"

Sequencer::Sequencer(std::vector<GuidanceActionTypes>& actionSequence, std::vector<Region>& regionSequence) : guidanceActionSequence() {
    generateGuidanceActionSequence(actionSequence, regionSequence);
}

void Sequencer::generateGuidanceActionSequence(std::vector<GuidanceActionTypes>& actionSequence, std::vector<Region>& regionSequence) {
    if(actionSequence.size() != regionSequence.size()) {
        ROS_ERROR("Lists of regions and actions to sequence are different sizes!");
    }

    //ROS_INFO("Length of region path: %d", regionSequence.size());
    int i = 0;
    while(i < actionSequence.size()) {
        auto type = actionSequence.at(i);
        std::vector<Region> currentActionVector;

        do {
            currentActionVector.push_back(regionSequence.at(i));
            i++;
        } while (i < actionSequence.size() && actionSequence.at(i-1) == actionSequence.at(i));

        guidanceActionSequence.push(Actions::makeGuidanceAction(type, currentActionVector));
    }

    ROS_INFO("Finished sequencing");
}

std::queue<std::shared_ptr<GuidanceAction>> Sequencer::getGuidanceActionSequence() {
    return guidanceActionSequence;
}