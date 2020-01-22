#pragma once 

#include "DataMessage.hpp"
#include "common_types.hpp"

enum class WaterFireExtState {Idle, Unarmed, Armed_Idle, Armed_Extinguishing, Armed_Extinguished, OutOfWater};

class WaterFireExtStateMsg : public DataMessage
{
public:

	msg_type getType();
	const int getSize();

	WaterFireExtState state;
};