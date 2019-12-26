#pragma once
#include "PVProvider.hpp"
#include "Pitch_PVProvider.hpp"
#include "Yaw_PVProvider.hpp"
#include "PitchRate_PVProvider.hpp"
#include "YawRate_PVProvider.hpp"
#include "NozzleOrientationMsg.hpp"
#include "ThreeAxisSensorMsg.hpp"
#include "logger.hpp"

class NozzleOrientationProvider : public Pitch_PVProvider, 
                                  public Yaw_PVProvider, 
                                  public PitchRate_PVProvider,
                                  public YawRate_PVProvider,
                                  public msg_receiver
{

private:

    AttitudeMsg m_att;
    HeadingMsg m_heading;
    Vector3D<float> m_rate;

public:

    //SFMPU9250_sensor* m_imu;

    NozzleOrientationProvider();
    AttitudeMsg getAttitude();
    //BodyRateProvider* getBodyRateProvider();
    Vector3D<float> getBodyRate();
    HeadingMsg getHeading();
    void receive_msg_data(DataMessage* t_msg);
};