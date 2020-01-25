#pragma once
#include <stdint.h>
#include "MsgEmitter.hpp"
#include "FloatMsg.hpp"
#include "WaterTankLevelMsg.hpp"

class PumpRosBridge : public msg_receiver, public msg_emitter
{
	void receive_msg_data(DataMessage*);
};