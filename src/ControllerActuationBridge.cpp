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
            case control_system::pitch_rate:
            {
                m_ctrl_output_msg.setControlSignal(control_system_msg->getData(), control_system::pitch_rate);
                this->emit_message((DataMessage*) &m_ctrl_output_msg);
                Logger::getAssignedLogger()->log("Pitch motor %f and PID %f ", m_ctrl_output_msg.getControlSignal(), pid_p ,LoggerLevel::Warning);
                break;
            }
            case control_system::yaw_rate:
            {
                m_ctrl_output_msg.setControlSignal(control_system_msg->getData(), control_system::yaw_rate);
                this->emit_message((DataMessage*) &m_ctrl_output_msg);
                Logger::getAssignedLogger()->log("Yaw motor %f and PID %f ", m_ctrl_output_msg.getControlSignal() , pid_y, LoggerLevel::Warning);
                break;
            }
            case control_system::pitch:
            {
                //Logger::getAssignedLogger()->log("Pitch PID %f: ", control_system_msg->getData(), LoggerLevel::Warning);
                pid_p = control_system_msg->getData();
                break;
            }
            case control_system::yaw:
            {
                //Logger::getAssignedLogger()->log("Yaw PID %f: ", control_system_msg->getData(), LoggerLevel::Warning);
                pid_y = control_system_msg->getData();
                break;
            }
            default:
                break;
            }
        }
    }
}