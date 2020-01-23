#include "SwitchMsg.hpp"

msg_type SwitchMsg::getType()
{
	return msg_type::SWITCH;
}

const int SwitchMsg::getSize()
{
	return sizeof(SwitchMsg);
}