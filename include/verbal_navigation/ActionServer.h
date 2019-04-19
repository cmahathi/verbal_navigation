#ifndef ACTION_SERVER_H
#define ACTION_SERVER_H

#include "TastyServer.h"
#include "verbal_navigation/Actions.h"
#include "verbal_navigation/ActionStream.h"
#include "verbal_navigation/Robot_Action.h"
#include <memory>
#include <vector>

using namespace std;

class ActionServer : public ActionStream {
public:
    ActionServer(string serverIP, port_t portNumber);

    // Sends action to client by indexing into clientConnections with the client_fd
    // int sendAction(client_fd client, Action action);
    int numClients();
    client_fd waitForClientConnection();

private:
    port_t masterPort;
    // The master server
    CTastyServer server;
    // The servers that manage individual clients
    vector<CTastyServer> clientConnections;
};

#endif