#pragma once 

#include "DataMessage.hpp"

class SwitchMsg : public DataMessage
{
public:

	msg_type getType();
	const int getSize();

	bool data;

};