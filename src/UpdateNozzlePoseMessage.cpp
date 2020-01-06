#include "UpdateNozzlePoseMessage.hpp"


UpdateNozzlePoseMessage::UpdateNozzlePoseMessage()
{
    _type = msg_type::UPDATEPOSEREFERENCE;
}

UpdateNozzlePoseMessage::~UpdateNozzlePoseMessage()
{

}

float UpdateNozzlePoseMessage::getPitch()
{
    return m_pitch;
}

float UpdateNozzlePoseMessage::getYaw()
{
    return m_yaw;
}

msg_type UpdateNozzlePoseMessage::getType()
{
    return _type;
}

void UpdateNozzlePoseMessage::setNozzlePoseMessage(float t_pitch, float t_yaw)
{
    m_pitch = t_pitch;
    m_yaw = t_yaw;
}

const int UpdateNozzlePoseMessage::getSize()
{
    return sizeof(UpdateNozzlePoseMessage);
}