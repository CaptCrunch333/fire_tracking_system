#include "TimedSwitch.hpp"

void TimedSwitch::setState(bool t_state)
{
    m_state = t_state;
    if(m_state)
    {
        m_timer.Start();
    }
    else
    {
        m_timer.Pause();
    }
    ControllerOutputMsg t_msg;
    t_msg.setControlSignal((float)m_state, control_system::pump);
    this->emit_message((DataMessage*) &t_msg);
}

bool TimedSwitch::getState()
{
    return m_state;
}

SwitchType TimedSwitch::getType()
{
    return SwitchType::TIMED_SWITCH;
}

float TimedSwitch::getRunningTime()
{
    return m_timer.getTime(TimeUnit::SECONDS);
}

void TimedSwitch::resetSwitch()
{
    m_timer.Stop();
    m_state = false;
    ControllerOutputMsg t_msg;
    t_msg.setControlSignal((float)m_state, control_system::pump);
    this->emit_message((DataMessage*) &t_msg);
}