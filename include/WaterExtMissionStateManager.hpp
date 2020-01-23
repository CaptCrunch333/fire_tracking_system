#pragma once
#include "MsgEmitter.hpp"
#include "IntegerMsg.hpp"
#include "WaterFireExtStateMsg.hpp"

class WaterExtMissionStateManager : public msg_emitter, public msg_receiver
{
public:
    void updateMissionState(WaterFireExtState);
    WaterFireExtState getMissionState();
    void receive_msg_data(DataMessage*);

private:
    WaterFireExtState current_outdoor_navigation_state = WaterFireExtState::Idle;
};

static WaterExtMissionStateManager waterExtMissionStateManager;