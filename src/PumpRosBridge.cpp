#include "PumpRosBridge.hpp"

void PumpRosBridge::receive_msg_data(DataMessage* t_msg)
{
    if(t_msg->getType() == msg_type::WATERTANKLEVEL)
    {
        FloatMsg t_float_msg;
        t_float_msg.data = ((WaterTankLevelMsg*) t_msg)->data;
        this->emit_message((DataMessage*) &t_float_msg);
    }
}