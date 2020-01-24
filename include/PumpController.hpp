#pragma once
#include "MsgEmitter.hpp"
#include "TimedSwitch.hpp"
#include "FloatMsg.hpp"
#include "WaterExtMissionStateManager.hpp"
#include "LUT2D.hpp"

class PumpController : public msg_receiver, public msg_emitter
{
    public:

        void setMaximumTriggeringDistance(float); //in meters
        void setHitboxSize(float);
        void setLookUpTable(LUT2D*);
        void receive_msg_data(DataMessage*);
        void add_callback_msg_receiver(msg_receiver* _callback_msg_receiver);
        
    private: 

        TimedSwitch m_switch;
        LUT2D* m_LUT;
        float m_TrigDist = 1;
};