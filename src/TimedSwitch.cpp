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
    SwitchMsg t_msg;
    t_msg.data = m_state;
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
    SwitchMsg t_msg;
    t_msg.data = m_state;
    this->emit_message((DataMessage*) &t_msg);
}