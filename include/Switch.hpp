#pragma once 

#include "MsgEmitter.hpp"

enum class SwitchType {TIMED_SWITCH};

class Switch : public msg_emitter
{

public:

	virtual void setState(bool) = 0;
	virtual bool getState() = 0;
	virtual SwitchType getType() = 0;
};