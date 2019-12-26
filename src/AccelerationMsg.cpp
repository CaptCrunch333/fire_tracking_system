#include "AccelerationMsg.hpp"

msg_type AccelerationMsg::getType()
{
	return msg_type::ACCELERATION;
}

const int AccelerationMsg::getSize()
{
	return sizeof(AccelerationMsg);
}