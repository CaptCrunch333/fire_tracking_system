#pragma once
#include "UserReference.hpp"
#include "ControlSystemMessage.hpp"
#include "UpdateNozzlePoseMessage.hpp"

class NozzleYaw_UserReference : public UserReference{

private:
    ControlSystemMessage m_output_msg;

public:

    void receive_msg_data(DataMessage*);
    
    NozzleYaw_UserReference();
    ~NozzleYaw_UserReference();
};