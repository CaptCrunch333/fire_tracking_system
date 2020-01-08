#pragma once

#include "MsgEmitter.hpp"
#include "comm_device.hpp"
#include "Timer.hpp"
#include "logger.hpp"

class CommChecker : public msg_emitter, public msg_receiver
{

public:

    CommChecker(CommDevice*, void*);
    ~CommChecker();

    void sendRequest(DataMessage*);
    void add_callback_msg_receiver(msg_receiver*);
    void receive_msg_data(DataMessage*);

private:

    const int m_timeout = 1000;
    bool status;
    Timer m_timer;
    CommDevice* m_dev;
    void* m_comm_port;

};