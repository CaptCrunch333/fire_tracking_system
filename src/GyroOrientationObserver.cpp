#include "GyroOrientationObserver.hpp"

GyroOrientationObserver::GyroOrientationObserver()
{
}

void GyroOrientationObserver::setFilterType(DataFilter* t_pitch_filter, DataFilter* t_roll_filter, DataFilter* t_yaw_filter)
{
    m_pitch_filter = t_pitch_filter;
    m_roll_filter = t_roll_filter;
    m_yaw_filter = t_yaw_filter;
}

void GyroOrientationObserver::updateSettings(FilterSettings* t_settings)
{
    m_pitch_filter->setFilterSettings(t_settings);
    m_roll_filter->setFilterSettings(t_settings);
    m_yaw_filter->setFilterSettings(t_settings);
}

void GyroOrientationObserver::loopInternal()
{
    m_filtered_attitude.pitch = m_pitch_filter->getFilteredData(gyro_data.x);
    m_filtered_attitude.roll = m_roll_filter->getFilteredData(gyro_data.y);
    m_filtered_heading.yaw = m_yaw_filter->getFilteredData(gyro_data.z);
    //std::cout << "gyro_angles: " << m_filtered_attitude.pitch << m_filtered_heading.yaw << std::endl;
    //Logger::getAssignedLogger()->log("gyro_angles: ", m_filtered_attitude.pitch, m_filtered_heading.yaw,LoggerLevel::Warning);
    //Logger::getAssignedLogger()->logtofile("gyro_angles: ", m_filtered_attitude.pitch, m_filtered_heading.yaw, LoggerLevel::Error);
    //std::cout << "getting called" << std::endl;
}

Vector3D<float> GyroOrientationObserver::getBodyRate()
{
    return gyro_data;
}

AttitudeMsg GyroOrientationObserver::getAttitude()
{
    return m_filtered_attitude;
}

HeadingMsg GyroOrientationObserver::getHeading()
{
    return m_filtered_heading;
}

void GyroOrientationObserver::receive_msg_data(DataMessage* t_msg)
{
    if(t_msg->getType() == msg_type::THREEAXISSENSORMSG)
    {
        ThreeAxisSensorMsg* t_sensor_msg = (ThreeAxisSensorMsg*) t_msg;
        gyro_data = t_sensor_msg->data;
        loopInternal();
    }
}