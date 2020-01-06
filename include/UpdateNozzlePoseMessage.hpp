#pragma once
#include "DataMessage.hpp"

class UpdateNozzlePoseMessage : public DataMessage
{

private:
    float m_pitch, m_yaw;
    msg_type _type;

public:

    float getPitch();
    float getYaw();
    msg_type getType();
    const int getSize();
    void setNozzlePoseMessage(float, float);

    UpdateNozzlePoseMessage();
    ~UpdateNozzlePoseMessage();
};