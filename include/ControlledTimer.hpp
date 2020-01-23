#pragma once
#include "Timer.hpp"

enum class TimeUnit {MICROSECONDS = 1, MILLISECONDS = 1000, SECONDS = 1000000};

class ControlledTimer : private Timer{
    
    public:
        void Start();
        void Stop();
        void Reset();
        void Pause();
        float getTime(TimeUnit);
    
    private:
        int m_AccumTime = 0;
        bool m_TimerRunning = false;
};