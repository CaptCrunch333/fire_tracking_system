#include "WaterExtMissionStateManager.hpp"

void WaterExtMissionStateManager::updateMissionState(WaterFireExtState t_current_state)
{
    current_outdoor_navigation_state = t_current_state;
    //WaterFireExtStateMsg state_msg;
    //state_msg.state = t_current_state;
    IntegerMsg state_msg;
    state_msg.data = (int)t_current_state;
    emit_message((DataMessage*) &state_msg);
}

WaterFireExtState WaterExtMissionStateManager::getMissionState()
{
    return current_outdoor_navigation_state;
}

void WaterExtMissionStateManager::receive_msg_data(DataMessage* t_msg)
{
    if(t_msg->getType() == msg_type::INTEGER)
    {
        updateMissionState((WaterFireExtState) ((IntegerMsg*)t_msg)->data);
    }
}