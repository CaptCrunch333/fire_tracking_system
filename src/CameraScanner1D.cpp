#include "CameraScanner1D.hpp"

void CameraScanner1D::receive_msg_data(DataMessage* t_msg) {
    
}

void CameraScanner1D::receive_msg_data(DataMessage* t_msg, int t_channel_id) {
    if(t_channel_id == (int)CAMERASCANNER_CHANNELS::DO_SWEEP) {
        this->doSweep();
    }
}

void CameraScanner1D::setDelay(int t_delay) {
    m_Timeout = t_delay;
}

void CameraScanner1D::doSweep() {
    ControllerOutputMsg t_msg;
    t_msg.setControlSignal(180, control_system::yaw);
    this->emit_message((DataMessage*) &t_msg);
    m_Timer.tick();
    while(m_Timer.tockMilliSeconds() < m_Timeout) {
        usleep(1000);
    }
    t_msg.setControlSignal(-180, control_system::yaw);
    m_Timer.tick();
    while(m_Timer.tockMilliSeconds() < m_Timeout) {
        usleep(1000);
    }
    t_msg.setControlSignal(0, control_system::yaw);
}