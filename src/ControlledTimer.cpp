#include "ControlledTimer.hpp"

void ControlledTimer::Start()
{
    if(!m_TimerRunning)
    {
        tick();
        m_TimerRunning = true;
    }
}

void ControlledTimer::Stop()
{
    m_AccumTime = 0;
    m_TimerRunning = false;
}

void ControlledTimer::Reset()
{
    m_AccumTime = 0;
    tick();
}

void ControlledTimer::Pause()
{
    if(m_TimerRunning)
    {
        m_AccumTime += tockMicroSeconds();
        m_TimerRunning = false;
    }
}

float ControlledTimer::getTime(TimeUnit t_unit)
{
    if(m_TimerRunning)
    {
        return (float)(m_AccumTime + tockMicroSeconds())/(float)t_unit;
    }
    else
    {
        return (float)m_AccumTime/(float)t_unit;
    }
}