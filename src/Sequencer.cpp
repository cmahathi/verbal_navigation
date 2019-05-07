#include "verbal_navigation/Sequencer.h"

Sequencer::Sequencer(std::vector<GuidanceActionTypes>& actionSequence, std::vector<Region>& regionSequence, ros::NodeHandle& nh) : guidanceActionSequence() {
    Actions::initializeClients(nh);
    generateGuidanceActionSequence(actionSequence, regionSequence);
}

void Sequencer::generateGuidanceActionSequence(std::vector<GuidanceActionTypes>& actionSequence, std::vector<Region>& regionSequence) {
    if(actionSequence.size() != regionSequence.size()) {
        ROS_ERROR("Lists of regions and actions to sequence are different sizes!");
    }

    //Put all sequential regions with the same action type into GuidanceActions
    int i = 0;
    while(i < actionSequence.size()) {
        auto type = actionSequence.at(i);
        std::vector<Region> currentActionVector;

        do {
            currentActionVector.push_back(regionSequence.at(i));
            ROS_INFO("Region: %s\tAction: %c", regionSequence.at(i).getName().c_str(), type);
            i++;
        } while (i < actionSequence.size() && actionSequence.at(i-1) == actionSequence.at(i) && actionSequence.at(i) != GuidanceActionTypes::TRANSITION);

        guidanceActionSequence.push(Actions::makeGuidanceAction(type, currentActionVector));
    }

    ROS_INFO("Finished sequencing");
}

std::queue<std::shared_ptr<GuidanceAction>> Sequencer::getGuidanceActionSequence() {
    return guidanceActionSequence;
}
