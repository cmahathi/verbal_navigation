#include "verbal_navigation/Sequencer.h"

Sequencer::Sequencer(std::vector<GuidanceActionTypes>& actionSequence, std::vector<Region>& regionSequence) : guidanceActionSequence() {
    generateGuidanceActionSequence(actionSequence, regionSequence);
}

void Sequencer::generateGuidanceActionSequence(std::vector<GuidanceActionTypes>& actionSequence, std::vector<Region>& regionSequence) {
    if(actionSequence.size() != regionSequence.size()) {
        ROS_ERROR("Lists of regions and actions to sequence are different sizes!");
    }

    //ROS_INFO("Length of region path: %d", regionSequence.size());
    GuidanceActionTypes type = actionSequence.at(0);
    std::vector<Region> currentActionVector;
    currentActionVector.push_back(regionSequence.at(0));
    for(int i = 1; i < actionSequence.size(); ++i) {
        //guidanceActionSequence.push(Actions::makeGuidanceAction(actionSequence.at(i), regionSequence.at(i)));
        if (type == actionSequence.at(i)) {
            currentActionVector.push_back(regionSequence.at(i));
            //ROS_INFO("%d: Added region %s to action", i, regionSequence.at(i).getName().c_str());
        }
        else {
            //ROS_INFO("%d: Creating new action starting with region %s", i, regionSequence.at(i).getName().c_str());
            std::vector<Region> v(currentActionVector);
            guidanceActionSequence.push(Actions::makeGuidanceAction(type, v));
            //ROS_INFO("Guidance action created");
            type = actionSequence.at(i);
            currentActionVector.clear();
            currentActionVector.push_back(regionSequence.at(i));
            //ROS_INFO("%d: Action created successfully.", i);
        }
    }
   // ROS_INFO("About to do last region");
    std::vector<Region> v(currentActionVector);
    guidanceActionSequence.push(Actions::makeGuidanceAction(type, v));
    ROS_INFO("Finished sequencing");
}

std::queue<std::shared_ptr<GuidanceAction>> Sequencer::getGuidanceActionSequence() {
    return guidanceActionSequence;
}