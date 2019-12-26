#include "ThreeAxisSensorMsg.hpp"

msg_type ThreeAxisSensorMsg::getType()
{
	return msg_type::THREEAXISSENSORMSG;
}

const int ThreeAxisSensorMsg::getSize()
{
	return sizeof(ThreeAxisSensorMsg);
}