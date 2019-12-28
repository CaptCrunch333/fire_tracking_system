#pragma once

#include "MsgEmitter.hpp"
#include "MsgReceiver.hpp"
#include "ControlSystemMessage.hpp"
#include "ControllerOutputMsg.hpp"
#include "logger.hpp"


class ControllerActuationBridge : public msg_emitter, public msg_receiver
{
private:

    ControllerOutputMsg m_ctrl_output_msg;
    float pid_p;
    float pid_y;

public:
    
    void receive_msg_data(DataMessage*);
};