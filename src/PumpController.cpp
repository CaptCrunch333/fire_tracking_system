#include "PumpController.hpp"

void PumpController::setMaximumTriggeringDistance(float t_val)
{
    m_TrigDist = t_val;
}

void PumpController::setHitboxSize(float)
{
    //TODO: Implement
}

void PumpController::setLookUpTable(LUT2D* t_LUT)
{
    m_LUT = t_LUT;
}

void PumpController::receive_msg_data(DataMessage* t_msg)
{
    FloatMsg* t_dist = (FloatMsg*) t_msg;
    if(t_msg->getType() == msg_type::FLOAT)
    {
        if(waterExtMissionStateManager.getMissionState() == WaterFireExtState::Armed_Idle)
        {
            if(m_LUT->getVal(m_switch.getRunningTime()) > 0)
            {
                if(t_dist->data <= m_TrigDist)
                {
                    m_switch.setState(true);
                    waterExtMissionStateManager.updateMissionState(WaterFireExtState::Armed_Extinguishing);
                }
            }
        }
        else if(waterExtMissionStateManager.getMissionState() == WaterFireExtState::Armed_Extinguishing)
        {
            if(m_LUT->getVal(m_switch.getRunningTime()) > 0)
            {
                if(t_dist->data > m_TrigDist)
                {
                    m_switch.setState(false);
                    waterExtMissionStateManager.updateMissionState(WaterFireExtState::Armed_Idle);
                }
            }
            else
            {
                m_switch.setState(false);
                waterExtMissionStateManager.updateMissionState(WaterFireExtState::OutOfWater);
            }
        }
        else if(m_switch.getState() == true)
        {
            m_switch.setState(false);
        }
    }
}

void PumpController::add_callback_msg_receiver(msg_receiver* t_rec)
{
    m_switch.add_callback_msg_receiver(t_rec);
}