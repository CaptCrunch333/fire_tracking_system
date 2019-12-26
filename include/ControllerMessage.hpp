#pragma once
#include "DataMessage.hpp"
#include <vector>
#include "PID_values.hpp"

class ControllerMessage : public DataMessage{

private:
    PID_data* _pid_data;
    msg_type _type;
    // PID_parameters* _pid_para;
    controller_msg_type _controller_msg_type;

public:
    PID_data* getData();
    const int getSize();
    // PID_parameters* getSettings();
    msg_type getType();
    controller_msg_type getControllerMsgType();

    ControllerMessage(controller_msg_type, PID_data*);
    // ControllerMessage(controller_msg_type, PID_parameters*);
    ~ControllerMessage();
};