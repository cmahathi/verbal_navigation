#include "verbal_navigation/Actions.h"

bool Actions::clientsInitialized = false;
ros::ServiceClient Actions::speech_client;
ros::ServiceClient Actions::go_to_location_client;