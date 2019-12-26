#pragma once
#include "UserReference.hpp"
#include "ControlSystemMessage.hpp"

class Yaw_UserReference : public UserReference{

private:
    ControlSystemMessage m_output_msg;

public:

    void receive_msg_data(DataMessage*);
    
    Yaw_UserReference();
    ~Yaw_UserReference();
};