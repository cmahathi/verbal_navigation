#include "verbal_navigation/Sequencer.h"

Sequencer::Sequencer(std::vector<GuidanceActionTypes>& actionSequence, std::vector<Region>& regionSequence) : guidanceActionSequence() {
    generateGuidanceActionSequence(actionSequence, regionSequence);
}

void Sequencer::generateGuidanceActionSequence(std::vector<GuidanceActionTypes>& actionSequence, std::vector<Region>& regionSequence) {
    if(actionSequence.size() != regionSequence.size()) {
        ROS_ERROR("Lists of regions and actions to sequence are different sizes!");
    }
    for(int i = 0; i < actionSequence.size(); ++i) {
        guidanceActionSequence.push(Actions::makeGuidanceAction(actionSequence.at(i), regionSequence.at(i)));
    }
}

std::queue<std::shared_ptr<GuidanceAction>> Sequencer::getGuidanceActionSequence() {
    return guidanceActionSequence;
}