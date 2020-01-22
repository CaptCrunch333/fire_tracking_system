#pragma once
#include "MsgEmitter.hpp"
#include "WaterFireExtStateMsg.hpp"

class MissionStateManager : public msg_emitter
{

public:
    void updateMissionState(WaterFireExtState);
    WaterFireExtState getMissionState();

private:
    WaterFireExtState current_outdoor_navigation_state = WaterFireExtState::Idle;
};

static MissionStateManager MainMissionStateManager;