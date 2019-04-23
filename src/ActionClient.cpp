#include "verbal_navigation/ActionClient.h"

ActionClient::ActionClient(string clientIP, string hostIP, port_t hostPort) : hostIP(hostIP), client(hostIP, hostPort) {
    establishLinkToHost(clientIP);
}
ActionClient::~ActionClient() {
    client.Close();
}

void ActionClient::establishLinkToHost(string clientIP) {
    ROS_INFO("Waiting for host");
    // Send IP to host and wait for connection
    size_t addressSize = clientIP.size();
    sendAll(&client, &addressSize, sizeof(size_t));
    auto IPMessage = convertToBuffer(clientIP);
    sendAll(&client, IPMessage, addressSize);
    
    // Recieve the port we are assigned and reestablish connection at that port
    auto myPort = receiveAll(&client, sizeof(port_t));

    port_t portNumber = *((port_t*)myPort->Buffer());
    client.Close();
    client = CTastyClient(hostIP, portNumber);
    ROS_INFO("Action client connected at %s:%d", hostIP.c_str(), portNumber);
}

verbal_navigation::Robot_Action ActionClient::waitForAction() {
    // Listen for how many bytes are in the message
    auto message = receiveAll(&client, sizeof(size_t));
    size_t bytesToReceive = *((size_t*)message->Buffer());

    // Receive the message and interpret it as a Robot_Action
    message = receiveAll(&client, bytesToReceive);
    auto action = *((verbal_navigation::Robot_Action*)message->Buffer());
    return action;
}