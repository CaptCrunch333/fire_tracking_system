#include "CommChecker.hpp"

CommChecker::CommChecker(CommDevice* t_dev, void* t_sender)
{
    m_timer.tick();
    m_dev = t_dev;
    m_comm_port = t_sender;
    while(1)
    {
        if(m_dev->attach_hardware_sender(m_comm_port))
        {
            Logger::getAssignedLogger()->log("Connection Established", LoggerLevel::Info);
            break;
        }
        else
        {
            Logger::getAssignedLogger()->log("Failed To Establish Connection, Retrying...", LoggerLevel::Info);
            //usleep(m_timeout - m_timer.tockMilliSeconds());
            m_timer.tick();
        }
    }
}

CommChecker::~CommChecker() 
{

}

void CommChecker::add_callback_msg_receiver(msg_receiver* t_rec)
{

}

void CommChecker::receive_msg_data(DataMessage* t_msg)
{
    this->emit_message(t_msg);
}