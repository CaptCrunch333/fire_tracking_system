#include "MissionStateManager.hpp"

void MissionStateManager::updateMissionState(WaterFireExtState t_current_state)
{
    current_outdoor_navigation_state = t_current_state;
    WaterFireExtStateMsg state_msg;
    state_msg.state = t_current_state;
    emit_message((DataMessage*) &state_msg);
}

WaterFireExtState MissionStateManager::getMissionState()
{
    return current_outdoor_navigation_state;
}
