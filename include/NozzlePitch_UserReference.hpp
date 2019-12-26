#pragma once
#include "UserReference.hpp"
#include "ControlSystemMessage.hpp"

class NozzlePitch_UserReference : public UserReference{

private:
    ControlSystemMessage m_output_msg;

public:

    void receive_msg_data(DataMessage*);
    
    NozzlePitch_UserReference();
    ~NozzlePitch_UserReference();
};