#pragma once 

#include "DataMessage.hpp"

class WaterTankLevelMsg : public DataMessage
{
public:

	msg_type getType();
	const int getSize();

	float data;

};