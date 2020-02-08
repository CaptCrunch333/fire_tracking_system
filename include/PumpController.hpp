#pragma once
#include "MsgEmitter.hpp"
#include "TimedSwitch.hpp"
#include "WaterExtMissionStateManager.hpp"
#include "LUT2D.hpp"
#include "FireAssessor.hpp"
#include "WaterEjectedMsg.hpp"
#include "EmptyMsg.hpp"
#include "FloatMsg.hpp"
#include "WaterTankLevelMsg.hpp"

class PumpController : public msg_receiver, public msg_emitter
{
    public:

        PumpController(TimedSwitch*, LUT2D*);
        void receive_msg_data(DataMessage*);
        
    private:

        TimedSwitch* m_switch;
        LUT2D* m_LUT;
        float m_prev_water_level = 0;
        float m_water_ejected = 0;
};