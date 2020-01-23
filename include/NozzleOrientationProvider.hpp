#pragma once
#include "Pitch_PVProvider.hpp"
#include "Roll_PVProvider.hpp"
#include "Yaw_PVProvider.hpp"
#include "DataFilter.hpp"
#include "NozzleOrientationMsg.hpp"
#include "ThreeAxisSensorMsg.hpp"
#include "logger.hpp"
#include "WaterExtMissionStateManager.hpp"

class NozzleOrientationProvider : public Pitch_PVProvider, 
                                  public Roll_PVProvider,
                                  public Yaw_PVProvider,
                                  public msg_receiver
{

public:

    NozzleOrientationProvider();
    void setFilterType(DataFilter*, DataFilter*, DataFilter*);
	void updateSettings(FilterSettings*);
	void receive_msg_data(DataMessage* t_msg);
	Vector3D<float> getBodyRate();
	AttitudeMsg getAttitude();
	HeadingMsg getHeading();


private:

    DataFilter* m_pitch_filter;
	DataFilter* m_roll_filter;
	DataFilter* m_yaw_filter;

    bool fire_state = false;

	AttitudeMsg m_filtered_attitude;
	HeadingMsg m_filtered_heading;
    Vector3D<float> cam_data;
	Vector3D<float> gyro_data;

	void loopInternal();


};