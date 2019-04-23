#ifndef ACTION_CLIENT_H
#define ACTION_CLIENT_H

#include "TastyClient.h"
#include "verbal_navigation/ActionStream.h"
#include "verbal_navigation/Robot_Action.h"

using namespace std;

class ActionClient : public ActionStream {
public:
    ActionClient(string clientIP, string hostIP, port_t hostPort);
    ~ActionClient();

private:
    string hostIP;
    CTastyClient client;
    void establishLinkToHost(string clientIP);
    verbal_navigation::Robot_Action waitForAction();
};

#endif