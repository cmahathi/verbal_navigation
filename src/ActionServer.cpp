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
    return clientConnections.size() - 1;
}

//Sends the size of the ros message, then the message itself
void ActionServer::sendActionToClient(std::shared_ptr<GuidanceAction> action, client_fd clientFD) {
    auto message = action->createMessage();
    size_t bytes = ros::serialization::serializationLength(message);
    auto client = &clientConnections.at(clientFD);
    sendAll(client, &bytes, sizeof(size_t));
    sendAll(client, &message, bytes);
}

void ActionServer::close(client_fd clientFD) {
    clientConnections.at(clientFD).Close();
}

int ActionServer::numClients() {
    return clientConnections.size();
}