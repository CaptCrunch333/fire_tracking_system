#include "NozzleYaw_UserReference.hpp"

NozzleYaw_UserReference::NozzleYaw_UserReference()
{
    
}

NozzleYaw_UserReference::~NozzleYaw_UserReference() {

}

void NozzleYaw_UserReference::receive_msg_data(DataMessage* t_msg){

    //TODO: fix, make command from obc
    m_output_msg.setControlSystemMessage(control_system::null_type, control_system_msg_type::to_system, 0);
    this->emit_message((DataMessage*) &m_output_msg);
}
