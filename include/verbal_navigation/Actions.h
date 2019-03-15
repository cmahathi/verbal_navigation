#ifndef ACTIONS_H
#define ACTIONS_H

#include <verbal_navigation/guidance_actions/GuidanceAction.h>
#include <verbal_navigation/guidance_actions/Lead.h>
#include <verbal_navigation/guidance_actions/Instruct.h>
#include <verbal_navigation/guidance_actions/Transition.h>

#include "verbal_navigation/Region.h"
#include <memory>

namespace Actions {
    //TODO Add ros::ServiceClient to constructor
    static std::shared_ptr<GuidanceAction> makeGuidanceAction(GuidanceActionTypes T, std::vector<Region> regions) {
        //ROS_INFO ("Creating Guidance Action");
        switch (T)
        {
            case GuidanceActionTypes::LEAD: {
                return std::make_shared<Lead>(regions);
            }
        
            case GuidanceActionTypes::INSTRUCT: {
                return std::make_shared<Instruct>(regions);
            }

            case GuidanceActionTypes::TRANSITION: {
                return std::make_shared<Transition>(regions.front());
            }

            default:
                ROS_ERROR("Attempt to construct GuidanceAction with invalid type");
                return std::shared_ptr<GuidanceAction>(nullptr);
        }
    }
}

#endif
