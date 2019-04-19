#include "verbal_navigation/ActionStream.h"

std::shared_ptr<TastyMessage> ActionStream::receiveAll(CTastyStream* stream, int numBytes) {
    auto message = std::make_shared<TastyMessage>(numBytes);
    TASTY_RC return_code;
    do{
        return_code = stream->Receive(*message);
    } while(return_code != TASTY_RC::TASTY_OK);

    return message;
}

void ActionStream::sendAll(CTastyStream* stream, void* buffer) {
    TastyMessage message((char*)buffer);
    TASTY_RC return_code;
    do{
        return_code = stream->Send(message);
    } while(return_code != TASTY_RC::TASTY_OK);
}

char* ActionStream::convertToBuffer(std::string str) {
    char* buff = new char[str.size()];
    str.copy(buff, str.size());
    return buff;
}