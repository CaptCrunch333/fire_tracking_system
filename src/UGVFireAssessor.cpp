#include "UGVFireAssessor.hpp"

void UGVFireAssessor::receive_msg_data(DataMessage* t_msg) {
    if(t_msg->getType() == msg_type::NOZZLEMSG)
    {
        // if(waterExtMissionStateManager.getMissionState()==WaterFireExtState::Idle)
        // {
        //     waterExtMissionStateManager.updateMissionState(WaterFireExtState::Unarmed);
        // }
        IntegerMsg t_int_msg;
        NozzleOrientationMsg* t_orient_msg = (NozzleOrientationMsg*)t_msg;
        if(t_orient_msg->fire_found == true)
        {
            // if(waterExtMissionStateManager.getMissionState()==WaterFireExtState::Unarmed) {
            //     waterExtMissionStateManager.updateMissionState(WaterFireExtState::Detected);
            // }
            //m_fire_detected = true;
            if(t_orient_msg->pitch <= m_ang_threshold && 
                t_orient_msg->yaw <= m_ang_threshold)
            {
                FireState t_state = FireState::DETECTED_INRANGE;
                t_int_msg.data = (int)t_state;
                this->emit_message((DataMessage*) &t_int_msg);
            }
            else
            {
                FireState t_state = FireState::DETECTED_OUTOFRANGE;
                t_int_msg.data = (int)t_state;
                this->emit_message((DataMessage*) &t_int_msg);
            }
        }
        else if(t_orient_msg->fire_found == false)
        {
            FireState t_state = FireState::NOTDETECTED;
            t_int_msg.data = (int)t_state;
            this->emit_message((DataMessage*) &t_int_msg);
            // if(m_fire_detected)
            // {
            //     if(m_timer.tockMilliSeconds() > m_Timeout)
            //     {
            //         if(m_water_ejected >= m_needed_water)
            //         {
            //             FireState t_state = FireState::EXTINGUISHED;
            //             t_int_msg.data = (int)t_state;
            //             this->emit_message((DataMessage*) &t_int_msg);
            //             waterExtMissionStateManager.updateMissionState(WaterFireExtState::Armed_Extinguished);
            //         }
            //         else
            //         {
            //             FireState t_state = FireState::DETECTED_OUTOFRANGE;
            //             t_int_msg.data = (int)t_state;
            //             this->emit_message((DataMessage*) &t_int_msg);
            //         }
            //     }
            // }
            // else
            // {
            //     FireState t_state = FireState::NOTDETECTED;
            //     t_int_msg.data = (int)t_state;
            //     this->emit_message((DataMessage*) &t_int_msg);
            // }
        }
    }
    // else if(t_msg->getType() == msg_type::WATEREJECTED)
    // {
    //     m_water_ejected = ((WaterEjectedMsg*) t_msg)->data;
    // }
    // else if(t_msg->getType() == msg_type::FLOAT)
    // {
    //     m_curr_distace = ((FloatMsg*) t_msg)->data;
    // }
}

void UGVFireAssessor::setAngleTolerance(float t_dim) {
    m_ang_threshold = t_dim;
}

// void UGVFireAssessor::setExtinguishedTimeout(int t_val) {
//     m_Timeout = t_val;
// }

// void UGVFireAssessor::setNeededWaterVolume(float t_val) {
//     m_needed_water = t_val;
// }

// void UGVFireAssessor::setMaximumTriggeringDistance(float t_val) {
//     m_TrigDist = t_val;
// }