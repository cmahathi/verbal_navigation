#ifndef ACTION_CLIENT_H
#define ACTION_CLIENT_H

#include "TastyClient.h"
#include "verbal_navigation/ActionStream.h"
#include "verbal_navigation/Actions.h"

using namespace std;

class ActionClient : public ActionStream {
public:
    ActionClient(string clientIP, string hostIP, port_t hostPort);
    ~ActionClient();

    ActionData waitForAction();
private:
    string hostIP;
    CTastyClient client;
    void establishLinkToHost(string clientIP);
};

#endif