#pragma once
#include "MsgEmitter.hpp"
#include "IntegerMsg.hpp"
#include "WaterFireExtStateMsg.hpp"
#include "logger.hpp"

class WaterExtMissionStateManager : public msg_emitter, public msg_receiver
{
public:
    void updateMissionState(WaterFireExtState);
    WaterFireExtState getMissionState();
    void receive_msg_data(DataMessage*);

private:
    WaterFireExtState current_water_ext_state = WaterFireExtState::Idle;
    void displayStateChange();
};

extern WaterExtMissionStateManager waterExtMissionStateManager;