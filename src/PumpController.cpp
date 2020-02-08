#include "PumpController.hpp"
#include <iostream>

PumpController::PumpController(TimedSwitch* t_switch, LUT2D* t_LUT)
{
    m_switch = t_switch;
    m_LUT = t_LUT;
}

void PumpController::receive_msg_data(DataMessage* t_msg)
{
    if(t_msg->getType() == msg_type::INTEGER)
    {
        FireState curr_fire_state = (FireState) ((IntegerMsg*) t_msg)->data;
        if(curr_fire_state == FireState::DETECTED_INRANGE)
        {
            if(waterExtMissionStateManager.getMissionState() == WaterFireExtState::Armed_Idle)
            {
                if(m_LUT->getVal(m_switch->getRunningTime()) > 0)
                {
                    m_switch->setState(true);
                    waterExtMissionStateManager.updateMissionState(WaterFireExtState::Armed_Extinguishing);
                }
            }
            else if(waterExtMissionStateManager.getMissionState() == WaterFireExtState::Armed_Extinguishing)
            {
                if(m_LUT->getVal(m_switch->getRunningTime()) > 0)
                {
                    float m_current_water_level = m_LUT->getVal(m_switch->getRunningTime());
                    m_water_ejected += m_prev_water_level - m_current_water_level;
                    m_prev_water_level = m_current_water_level;
                    WaterEjectedMsg t_water_level_msg;
                    t_water_level_msg.data = m_water_ejected;
                    this->emit_message((DataMessage*) &t_water_level_msg);
                }
                else
                {
                    m_switch->setState(false);
                    waterExtMissionStateManager.updateMissionState(WaterFireExtState::OutOfWater);
                }
            }
        }
        else if(curr_fire_state == FireState::DETECTED_OUTOFRANGE)
        {
            if(waterExtMissionStateManager.getMissionState() == WaterFireExtState::Armed_Extinguishing)
            {
                m_switch->setState(false);
                waterExtMissionStateManager.updateMissionState(WaterFireExtState::Armed_Idle);
            }
        }
        else if(curr_fire_state == FireState::EXTINGUISHED)
        {
            Logger::getAssignedLogger()->log("Fire Successfully Extinguished - Turning Pump Off", LoggerLevel::Info);
            m_switch->setState(false);
        }
        else if(curr_fire_state == FireState::NOTDETECTED)
        {
            if(waterExtMissionStateManager.getMissionState() == WaterFireExtState::Idle)
            {
                Logger::getAssignedLogger()->log("Reseting Water Level", LoggerLevel::Info);
                m_switch->resetSwitch();
            }
        }
    }
    else if(t_msg->getType() == msg_type::EMPTY)
    {
        WaterTankLevelMsg t_float_msg;
        t_float_msg.data = m_LUT->getVal(m_switch->getRunningTime());
        this->emit_message((DataMessage*) &t_float_msg);
    }
}