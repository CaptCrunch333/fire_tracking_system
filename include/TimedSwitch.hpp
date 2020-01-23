#pragma once 

#include "Switch.hpp"
#include "ControlledTimer.hpp"
#include "IntegerMsg.hpp"
#include "logger.hpp"

class TimedSwitch : public Switch
{
	
public:

	void setState(bool);
	bool getState();
	SwitchType getType();
	float getRunningTime(); // in secs
	void resetSwitch();

private:

	ControlledTimer m_timer;
	bool m_state = true;
};