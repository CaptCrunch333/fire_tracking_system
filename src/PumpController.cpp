#include "PumpController.hpp"

void PumpController::setMaximumTriggeringDistance(float t_val)
{
    m_TrigDist = t_val;
}
void PumpController::setHitboxSize(float)
{
    //TODO: Implement
}
void PumpController::receive_msg_data(DataMessage* t_msg)
{
    if(t_msg->getType() == msg_type::FLOAT)
    {
        if(waterExtMissionStateManager.getMissionState() >= WaterFireExtState::Armed_Idle)
        {
            FloatMsg* t_dist = (FloatMsg*) t_msg;
            if()
            {
                
            }
            else if(t_dist->data > m_TrigDist)
            {
                m_switch.setState(false);
                waterExtMissionStateManager.updateMissionState(WaterFireExtState::Armed_Idle);
            }
        }
    }
}

void PumpController::add_callback_msg_receiver(msg_receiver* t_rec)
{
    m_switch.add_callback_msg_receiver(t_rec);
}