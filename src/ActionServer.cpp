#include "verbal_navigation/ActionServer.h"

ActionServer::ActionServer(string serverIP, unsigned int portNumber) : server(serverIP, portNumber), masterPort(portNumber) {
    ROS_INFO("ActionServer started  on %s:%d with code %d", serverIP, portNumber, server.InitializeServerIfNecessary());
}

client_fd ActionServer::waitForClientConnection() {
    // Wait for client to connect
    TastyMessage clientIPMessage(100);
    while(!server.IsConnected());
    server.Receive(clientIPMessage);

    // Assign a mew server to the client
    port_t clientPort = masterPort + clientConnections.size() + 1;
    clientConnections.emplace(clientIP, clientPort);

    // Send the client its port assignment. Kind of want to modify TastyMessage to use void* as buffer
    TastyMessage clientPortMessage(clientPort);
    clientConnections.at(clientConnections.size()).Send(clientPortMessage);
    return clientConnections.size();
}