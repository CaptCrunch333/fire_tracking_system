#include "LUT2D.hpp"
#define DEBUG_LUT
void LUT2D::setLUT(std::string t_path)
{
    std::ifstream t_file;
    t_file.open(t_path);
    if (t_file.is_open())
    {
        int t_array_index = 0;
        while(!t_file.eof())
        {
            t_file >> m_LUT[t_array_index][0] >> m_LUT[t_array_index][1];
            t_array_index++;
        }
        m_length = t_array_index - 1;
        t_file.close();
        #ifdef DEBUG_LUT
        Logger::getAssignedLogger()->log("Lut size: %f", (float)m_length, LoggerLevel::Info);
        for(int i = 0; i < m_length; i++)
        {
            Logger::getAssignedLogger()->log("LUT ROW: %f, %f", m_LUT[i][0], m_LUT[i][1], LoggerLevel::Info);
        }
        #endif
    }
    else
    {
        Logger::getAssignedLogger()->log("Couldn't Open LUT File", LoggerLevel::Warning);
    }
}

float LUT2D::getVal(float t_time)
{
    if(t_time < 0)
    {
        return m_LUT[0][1];
    }
    else
    {
        for(int i = 0; i < m_length; i++)
        {
            if(m_LUT[i][0] == t_time)
            {
                return m_LUT[i][1];
            }
            else if(m_LUT[i][0] > t_time)
            {
                float t1 = m_LUT[i-1][0];
                float t2 = m_LUT[i][0];
                float weight1 = (t2 - t_time)/(t2 - t1);
                float weight2 = (t_time - t1)/(t2 - t1);
                return m_LUT[i-1][1]*weight1 + m_LUT[i][1]*weight2;
            }
        }
        m_LUT[m_length - 1][1];
    }
}