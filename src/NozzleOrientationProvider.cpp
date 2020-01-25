#include "NozzleOrientationProvider.hpp"

NozzleOrientationProvider::NozzleOrientationProvider()
{

}

void NozzleOrientationProvider::setFilterType(DataFilter* t_pitch_filter, DataFilter* t_roll_filter, DataFilter* t_yaw_filter)
{
    m_pitch_filter = t_pitch_filter;
    m_roll_filter = t_roll_filter;
    m_yaw_filter = t_yaw_filter;
}

void NozzleOrientationProvider::updateSettings(FilterSettings* t_settings)
{
    m_pitch_filter->setFilterSettings(t_settings);
    m_roll_filter->setFilterSettings(t_settings);
    m_yaw_filter->setFilterSettings(t_settings);
}

void NozzleOrientationProvider::loopInternal()
{
    if(waterExtMissionStateManager.getMissionState() == WaterFireExtState::Armed_Idle ||
        waterExtMissionStateManager.getMissionState() == WaterFireExtState::Armed_Extinguishing)
    {
        m_filtered_attitude.pitch = m_pitch_filter->getFilteredData(cam_data.x, gyro_data.x);
        //m_filtered_attitude.roll = m_roll_filter->getFilteredData(gyro_data.y);
        m_filtered_heading.yaw = m_yaw_filter->getFilteredData(cam_data.z, gyro_data.z);

        //Logger::getAssignedLogger()->log("observer angles: %f , %f", m_filtered_attitude.pitch, m_filtered_heading.yaw,LoggerLevel::Warning);
        //Logger::getAssignedLogger()->logtofile("gyro_angles: ", m_filtered_attitude.pitch, m_filtered_heading.yaw, LoggerLevel::Error);
    }
}

AttitudeMsg NozzleOrientationProvider::getAttitude()
{
    return m_filtered_attitude;
}

HeadingMsg NozzleOrientationProvider::getHeading()
{
    return m_filtered_heading;
}

Vector3D<float> NozzleOrientationProvider::getBodyRate()
{
    return gyro_data;
}

void NozzleOrientationProvider::receive_msg_data(DataMessage* t_msg)
{
    if(t_msg->getType() == msg_type::NOZZLEMSG)
    {
        NozzleOrientationMsg* t_nozzle_msg = (NozzleOrientationMsg*) t_msg;
        if(t_nozzle_msg->fire_found)
        {
            cam_data.x = t_nozzle_msg->pitch;
            cam_data.z = t_nozzle_msg->yaw;
            if(!fire_state)
            {
                m_pitch_filter->resetFilter(t_nozzle_msg->pitch);
                m_yaw_filter->resetFilter(t_nozzle_msg->yaw);
                fire_state = t_nozzle_msg->fire_found;
            }
        }
        else
        {
            cam_data = 0;
            m_pitch_filter->resetFilter(0);
            m_yaw_filter->resetFilter(0);
            fire_state = false;
        }
        
        //Logger::getAssignedLogger()->logtofile("cam_angles: ", cam_data.x, cam_data.z, LoggerLevel::Error);
    }
    else if(t_msg->getType() == msg_type::THREEAXISSENSORMSG)
    {
        ThreeAxisSensorMsg* t_rate_msg = (ThreeAxisSensorMsg*) t_msg;
        if(fire_state)
        {
            gyro_data = t_rate_msg->data * -1.f;
        }
        else
        {
            gyro_data = 0;
        }
        loopInternal();
        //Logger::getAssignedLogger()->logtofile("gyro_angles: ", gyro_data.x, gyro_data.z, LoggerLevel::Error);
    }
}