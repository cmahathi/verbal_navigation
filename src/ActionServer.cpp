#include "verbal_navigation/ActionServer.h"

ActionServer::ActionServer(string serverIP, unsigned int portNumber) : serverIP(serverIP), server(serverIP, portNumber), masterPort(portNumber) {
    ROS_INFO("ActionServer started  on %s:%d with code %d", serverIP.c_str(), portNumber, server.InitializeServerIfNecessary());
}

client_fd ActionServer::waitForClientConnection() {
    // Wait for client to connect
    ROS_INFO("Waiting for client");
    size_t incomingSize = *((size_t*)receiveAll(&server, sizeof(size_t))->Buffer());

    string clientIP = receiveAll(&server, incomingSize)->Buffer();
    ROS_INFO("SERVER: Client connected from %s", clientIP.c_str());

    // Assign a new server to the client
    port_t clientPort = masterPort + clientConnections.size() + 1;
    clientConnections.emplace_back(serverIP, clientPort);

    // Send the client its port assignment.
    sendAll(&server, &clientPort, sizeof(port_t));
    return clientConnections.size();
}

int ActionServer::numClients() {
    return clientConnections.size();
}