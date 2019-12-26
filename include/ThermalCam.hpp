#pragma once

#include "CamSpecs.hpp"
#include "ImageConverter.hpp"
#include "ThermalImageMsg.hpp"
#include "positioning_system/temp_range.h"
#include "MsgReceiver.hpp"
#include "MsgEmitter.hpp"

enum camera_name {
	LEPTON3_5 // 0
};

enum scaling_method {
	FIXED_SCALING, // 0
	AUTO_SCALING  // 1
};

class ThermalCam : public msg_receiver, public msg_emitter
{

public:
	virtual CamSpecs getSpecs() = 0;
	virtual void receive_msg_data(DataMessage*) = 0;
};