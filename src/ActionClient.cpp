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
