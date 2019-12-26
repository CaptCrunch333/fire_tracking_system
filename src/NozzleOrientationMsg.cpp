#include "NozzleOrientationMsg.hpp"

msg_type NozzleOrientationMsg::getType()
{
	return msg_type::NOZZLEMSG;
}

const int NozzleOrientationMsg::getSize()
{
	return sizeof(NozzleOrientationMsg);
}