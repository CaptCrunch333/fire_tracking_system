#include "Negator.hpp"

void Negator::receive_msg_data(DataMessage* t_msg) {
    if(t_msg->getType() == msg_type::THREEAXISSENSORMSG) {
        ThreeAxisSensorMsg* t_3_axis_msg = (ThreeAxisSensorMsg*) t_msg;
        t_3_axis_msg->data.x = -1 * t_3_axis_msg->data.x;
        t_3_axis_msg->data.y = -1 * t_3_axis_msg->data.y;
        t_3_axis_msg->data.z = -1 * t_3_axis_msg->data.z;
        emit_message((DataMessage*) t_3_axis_msg);
    }
    else if(t_msg->getType() == msg_type::NOZZLEMSG) {
        NozzleOrientationMsg* t_nozzle_msg = (NozzleOrientationMsg*) t_msg;
        t_nozzle_msg->pitch = -1 * t_nozzle_msg->pitch;
        t_nozzle_msg->yaw = -1 * t_nozzle_msg->yaw;
        emit_message((DataMessage*) t_nozzle_msg);
    }
}