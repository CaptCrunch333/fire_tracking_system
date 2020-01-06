#include "ControllerActuationBridge.hpp"

void ControllerActuationBridge::receive_msg_data(DataMessage* t_msg)
{
    if(t_msg->getType() == msg_type::control_system)
    {
        ControlSystemMessage* control_system_msg = (ControlSystemMessage*)t_msg;
        if(control_system_msg->getControlSystemMsgType() == control_system_msg_type::to_system)
        {
            switch (control_system_msg->getSource())
            {
            case control_system::pitch:
            {
                m_ctrl_output_msg.setControlSignal(control_system_msg->getData(), control_system::pitch);
                this->emit_message((DataMessage*) &m_ctrl_output_msg);
                Logger::getAssignedLogger()->log("Pitch motor %f and PID %f ", m_ctrl_output_msg.getControlSignal() ,LoggerLevel::Warning);
                //Logger::getAssignedLogger()->logtofile("pitch_motor: ", m_ctrl_output_msg.getControlSignal() ,LoggerLevel::Error);
                break;
            }
            case control_system::yaw:
            {
                m_ctrl_output_msg.setControlSignal(control_system_msg->getData(), control_system::yaw);
                this->emit_message((DataMessage*) &m_ctrl_output_msg);
                Logger::getAssignedLogger()->log("Yaw motor %f and PID %f ", m_ctrl_output_msg.getControlSignal(), LoggerLevel::Warning);
                //Logger::getAssignedLogger()->logtofile("yaw_motor: ", m_ctrl_output_msg.getControlSignal() ,LoggerLevel::Error);
                break;
            }
            default:
                break;
            }
        }
    }
}