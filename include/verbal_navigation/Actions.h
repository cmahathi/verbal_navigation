#ifndef ACTIONS_H
#define ACTIONS_H

#include <verbal_navigation/guidance_actions/ActionData.h>
#include <verbal_navigation/guidance_actions/GuidanceAction.h>
#include <verbal_navigation/guidance_actions/Lead.h>
#include <verbal_navigation/guidance_actions/Instruct.h>
#include <verbal_navigation/guidance_actions/Transition.h>
#include "verbal_navigation/Wavenet.h"

#include "verbal_navigation/Region.h"
#include <memory>

// Factory class for GuidanceActions

class Actions {
    static bool clientsInitialized;
    static ros::ServiceClient speech_client;
    static ros::ServiceClient go_to_location_client;
    static ros::ServiceClient goToLocationClient;

public:
    static void initializeClients(ros::NodeHandle& node) {
        if(!clientsInitialized) {
            // speech_client = node.serviceClient <verbal_navigation::Wavenet> ("/wavenet");
            // speech_client.waitForExistence();
            // ROS_INFO("Speech Client Found!");
            // ROS_INFO("Location client unimplemented");
            // // speech_client->waitForExistence();
            // // go_to_location_client = new node.serviceClient <gotolocation>;
            // // go_to_location_client->waitForExistence();
            // clientsInitialized = true;
        }
    }

    static std::shared_ptr<GuidanceAction> makeGuidanceAction(GuidanceActionTypes T, std::vector<Region> regions) {
        // if(!clientsInitialized) {
        //     ROS_ERROR("Attempt to construct GuidanceActions before clients are initialized!");
        //     return std::shared_ptr<GuidanceAction>(nullptr);
        // }

        switch (T)
        {
            case GuidanceActionTypes::LEAD: {
                return std::make_shared<Lead>(regions, go_to_location_client);
            }
        
            case GuidanceActionTypes::INSTRUCT: {
                return std::make_shared<Instruct>(regions, speech_client);
            }

            case GuidanceActionTypes::TRANSITION: {
                return std::make_shared<Transition>(regions.front(), speech_client);
            }

            default:
                ROS_ERROR("Attempt to construct GuidanceAction with invalid type");
                return std::shared_ptr<GuidanceAction>(nullptr);
        }
    }
};

#endif
