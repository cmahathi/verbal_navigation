#include "verbal_navigation/ActionServer.h"

ActionServer::ActionServer(string serverIP, unsigned int portNumber) : server(serverIP, portNumber), masterPort(portNumber) {
    ROS_INFO("ActionServer started  on %s:%d with code %d", serverIP.c_str(), portNumber, server.InitializeServerIfNecessary());
}

client_fd ActionServer::waitForClientConnection() {
    // Wait for client to connect
    int incomingSize = *((int*)receiveAll(&server, sizeof(int))->Buffer());
    string clientIP = receiveAll(&server, incomingSize)->Buffer();
    ROS_INFO("SERVER: Client connected from %s", clientIP);

    // Assign a new server to the client
    port_t clientPort = masterPort + clientConnections.size() + 1;
    clientConnections.emplace_back(clientIP, clientPort);

    // Send the client its port assignment. Kind of want to modify TastyMessage to use void* as buffer
    sendAll(&clientConnections.at(clientConnections.size()), &clientPort);
    return clientConnections.size();
}

int ActionServer::numClients() {
    return clientConnections.size();
}