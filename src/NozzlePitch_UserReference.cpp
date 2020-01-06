#include "NozzlePitch_UserReference.hpp"

NozzlePitch_UserReference::NozzlePitch_UserReference()
{
    
}

NozzlePitch_UserReference::~NozzlePitch_UserReference()
{

}

void NozzlePitch_UserReference::receive_msg_data(DataMessage* t_msg)
{
    if(t_msg->getType() == msg_type::UPDATENOZZLEPOSEREFERENCE){   

        UpdateNozzlePoseMessage* user_msg = (UpdateNozzlePoseMessage*)t_msg;
        m_output_msg.setControlSystemMessage(control_system::null_type, control_system_msg_type::to_system, user_msg->getPitch());
        this->emit_message((DataMessage*) &m_output_msg);
    }
}