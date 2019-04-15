#ifndef ACTION_CLIENT_H
#define ACTION_CLIENT_H

#include "TastyClient.h"
#include "verbal_navigation/ActionServer.h"
#include "verbal_navigation/Robot_Action.h"

using namespace std;

class ActionClient {
public:
    ActionClient(string clientIP, string hostIP, port_t hostPort);

private:
    CTastyClient client;
    establishLinkToHost(string clientIP);
    Robot_Action waitForAction();
};

#endif