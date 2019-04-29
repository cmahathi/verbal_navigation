#ifndef ACTION_STREAM_H
#define ACTION_STREAM_H

#include "TastyStream.h"
#include <memory>
#include <ros/ros.h>

#define client_fd unsigned int
#define port_t unsigned int

class ActionStream {
    protected:
    std::shared_ptr<TastyMessage> receiveAll(CTastyStream* stream, int numBytes);
    void sendAll(CTastyStream* stream, void* buffer, size_t bytes);
    char* convertToBuffer(std::string str);
    size_t receiveNextMessageSize(CTastyStream* stream);
};

#endif