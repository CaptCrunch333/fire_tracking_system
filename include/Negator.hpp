#pragma once

#include "MsgEmitter.hpp"
#include "ThreeAxisSensorMsg.hpp"
#include "NozzleOrientationMsg.hpp"

class Negator : public msg_receiver, public msg_emitter
{
    void receive_msg_data(DataMessage*);
};