#pragma once 

#include "DataMessage.hpp"
#include "common_types.hpp"

class AccelerationMsg : public DataMessage
{
public:

	msg_type getType();
	const int getSize();

	float ddx, ddy, ddz;	
};