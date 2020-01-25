#pragma once 

#include "DataMessage.hpp"
#include "common_types.hpp"
enum class WaterFireExtState {Error = -1, Idle, Unarmed, Armed_Idle, Armed_Extinguishing, Armed_Extinguished, OutOfWater};
#include "WaterExtMissionStateManager.hpp"

class WaterFireExtStateMsg : public DataMessage
{
public:

	msg_type getType();
	const int getSize();

	WaterFireExtState state;
};