#pragma once 

#include <unistd.h>
#include "MsgEmitter.hpp"
#include "Timer.hpp"
#include "ControllerOutputMsg.hpp"

enum class CAMERASCANNER_CHANNELS {DO_SWEEP = 1};

class CameraScanner1D : public msg_receiver, public msg_emitter 
{
	public:
		msg_type getType();
		void receive_msg_data(DataMessage*);
		void receive_msg_data(DataMessage*, int);
		void setDelay(int); // Delay in ms

	private:
		Timer m_Timer;
		int m_Timeout;
		void doSweep();

};