#pragma once

#include "MsgEmitter.hpp"
#include "FloatMsg.hpp"
#include "WaterEjectedMsg.hpp"
#include "Timer.hpp"
#include "IntegerMsg.hpp"
#include "NozzleOrientationMsg.hpp"
#include "WaterExtMissionStateManager.hpp"
#include "FireAssessmentStatus.hpp"

class FireAssessor : public msg_emitter, public msg_receiver
{
    public:
        void receive_msg_data(DataMessage*);
        void setAngleTolerance(float); //in Rads
        void setExtinguishedTimeout(int); //in millisecs
        void setMaximumTriggeringDistance(float); //in meters
        void setNeededWaterVolume(float); //in LUT values

    private:
        Timer m_timer;
        bool m_fire_detected;
        int m_Timeout = 2000;
        float m_ang_threshold;
        float m_needed_water = 1;
        float m_water_ejected = 0;
        float m_TrigDist = 1;
        float m_curr_distace;
};