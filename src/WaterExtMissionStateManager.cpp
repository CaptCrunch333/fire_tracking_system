#include "WaterExtMissionStateManager.hpp"

WaterExtMissionStateManager waterExtMissionStateManager;

void WaterExtMissionStateManager::updateMissionState(WaterFireExtState t_current_state)
{
    current_water_ext_state = t_current_state;
    IntegerMsg state_msg;
    state_msg.data = (int)t_current_state;
    emit_message((DataMessage*) &state_msg);
    displayStateChange();
}

WaterFireExtState WaterExtMissionStateManager::getMissionState()
{
    return current_water_ext_state;
}

void WaterExtMissionStateManager::receive_msg_data(DataMessage* t_msg)
{
    if(t_msg->getType() == msg_type::INTEGER)
    {
        updateMissionState((WaterFireExtState) ((IntegerMsg*)t_msg)->data);
    }
}

void WaterExtMissionStateManager::displayStateChange()
{
    switch (current_water_ext_state)
    {

    case WaterFireExtState::Error:
        Logger::getAssignedLogger()->log("Water Ext State Changed To Error", LoggerLevel::Info);
        break;

    case WaterFireExtState::Idle:
        Logger::getAssignedLogger()->log("Water Ext State Changed To Idle", LoggerLevel::Info);
        break;

    case WaterFireExtState::Unarmed:
        Logger::getAssignedLogger()->log("Water Ext State Changed To Unarmed", LoggerLevel::Info);
        break;

    case WaterFireExtState::Armed_Idle:
        Logger::getAssignedLogger()->log("Water Ext State Changed To Armed_Idle", LoggerLevel::Info);
        break;
    
    case WaterFireExtState::Armed_Extinguishing:
        Logger::getAssignedLogger()->log("Water Ext State Changed To Armed_Extinguishing", LoggerLevel::Info);
        break;

    case WaterFireExtState::Armed_Extinguished:
        Logger::getAssignedLogger()->log("Water Ext State Changed To Armed_Extinguished", LoggerLevel::Info);
        break;

    case WaterFireExtState::OutOfWater:
        Logger::getAssignedLogger()->log("Water Ext State Changed To OutOfWater", LoggerLevel::Info);
        break;

    default:
        break;
    }
}