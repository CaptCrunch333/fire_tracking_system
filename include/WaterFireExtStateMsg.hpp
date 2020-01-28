#pragma once 

#include "DataMessage.hpp"
#include "common_types.hpp"
#include "WaterFireExtState.hpp"
#include "WaterExtMissionStateManager.hpp"

class WaterFireExtStateMsg : public DataMessage
{
public:

	msg_type getType();
	const int getSize();

	WaterFireExtState state;
};