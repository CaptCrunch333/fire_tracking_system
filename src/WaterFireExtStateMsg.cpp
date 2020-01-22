#include "WaterFireExtStateMsg.hpp"

msg_type WaterFireExtStateMsg::getType()
{
	return msg_type::WATEREXTSTATE;
}

const int WaterFireExtStateMsg::getSize()
{
	return sizeof(WaterFireExtStateMsg);
}