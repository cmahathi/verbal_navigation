#include "verbal_navigation/ActionClient.h"

ActionClient::ActionClient(string clientIP, string hostIP, port_t hostPort) : hostIP(hostIP), client(hostIP, hostPort) {
    establishLinkToHost(clientIP);
}

void ActionClient::establishLinkToHost(string clientIP) {
    // Send IP to host and wait for connection
    auto addressSize = clientIP.size();
    sendAll(&client, &addressSize);
    auto IPMessage = convertToBuffer(clientIP);
    sendAll(&client, IPMessage);
    delete IPMessage;
    
    // Recieve the port we are assigned and reestablish connection at that port
    auto myPort = receiveAll(&client, sizeof(port_t));
    while(!client.IsConnected());
    port_t portNumber = *((port_t*)myPort->Buffer());
    client = CTastyClient(hostIP, portNumber);
    ROS_INFO("Action client connected at %s:%d", hostIP.c_str(), portNumber);
}