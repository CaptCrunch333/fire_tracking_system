#include "ThermalImageMsg.hpp"

msg_type ThermalImageMsg::getType()
{
	return msg_type::THERMALIMAGE;
}

const int ThermalImageMsg::getSize()
{
	return sizeof(ThermalImageMsg);
}