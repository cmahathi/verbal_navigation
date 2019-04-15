#include "verbal_navigation/ActionClient.h"

ActionClient::ActionClient(string clientIP, string hostIP, port_t hostPort) : client(hostIP, hostPort) {
    establishLinkToHost(clientIP);
}

ActionClient::establishLinkToHost(string clientIP) {
    // Send IP to host and wait for connection
    TastyMessage myIP(clientIP);
    client.Send(myIP);
    while(!client.IsConnected());

    // Recieve the port we are assigned and reestablish connection at that port
    TastyMessage myPort(sizeof(port_t));
    client.Receive(myPort);
    port_t portNumber = *((port_t*)myPort.Buffer());
    client = CTastyClient(hostIP, portNumber);
    ROS_INFO("Action client connected at %s:%d", hostIP, portNumber);
}

int ActionClient::numClients() {
    return clientConnections.size();
}