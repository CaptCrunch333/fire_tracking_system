#include "ControllerMessage.hpp"

ControllerMessage::ControllerMessage(controller_msg_type t_type, PID_data* t_data) {
    _type = msg_type::controller;
    _controller_msg_type = t_type;
    _pid_data = t_data;
}

// ControllerMessage::ControllerMessage(controller_msg_type t_type, PID_parameters* t_pid_para) {
//     _type = msg_type::controller;
//     _controller_msg_type = t_type;
//     _pid_para = t_pid_para;
// }

ControllerMessage::~ControllerMessage() {

}

PID_data* ControllerMessage::getData(){
    return _pid_data;
}
// PID_parameters* ControllerMessage::getSettings(){
//     return _pid_para;
// }
msg_type ControllerMessage::getType(){
    return _type;
}

const int ControllerMessage::getSize()
{
    return sizeof(this);
}

controller_msg_type ControllerMessage::getControllerMsgType(){
    return _controller_msg_type;
}