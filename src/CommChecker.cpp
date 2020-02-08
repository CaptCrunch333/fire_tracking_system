#include "CommChecker.hpp"

CommChecker::CommChecker(CommDevice* t_dev, void* t_sender, block_frequency t_bf) : TimedBlock(t_bf)
{
    m_timer.tick();
    m_dev = t_dev;
    m_comm_port = t_sender;
    if(m_dev->attach_hardware_sender(m_comm_port))
    {
        Logger::getAssignedLogger()->log("Connection Established", LoggerLevel::Info);
    }
    else
    {
        Logger::getAssignedLogger()->log("Failed To Establish Connection...", LoggerLevel::Info);
        waterExtMissionStateManager.updateMissionState(WaterFireExtState::Error);
    }
}

CommChecker::~CommChecker() 
{

}

void CommChecker::loopInternal()
{
    m_timer.tick(); //TODO: DELETE
    HeartbeatMsg t_msg;
    emit_message((DataMessage*) &t_msg);
    if(m_timer.tockMicroSeconds() >= m_timeout)
    {
        Logger::getAssignedLogger()->log("Communication with device lost, please check Connection", LoggerLevel::Info);
        m_dev->reset_hardware(m_comm_port);
        waterExtMissionStateManager.updateMissionState(WaterFireExtState::Error);
        m_timer.tick();
    }
}

void CommChecker::receive_msg_data(DataMessage* t_msg)
{
    //Logger::getAssignedLogger()->log("hearbeat rec", LoggerLevel::Info);
    if(t_msg->getType() == msg_type::HEARTBEAT)
    {
        if(waterExtMissionStateManager.getMissionState() == WaterFireExtState::Error)
        {
            waterExtMissionStateManager.updateMissionState(WaterFireExtState::Unarmed);
        }
        m_timer.tick();
    }
}