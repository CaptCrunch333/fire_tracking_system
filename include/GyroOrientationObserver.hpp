#pragma once 

#include "DataFilter.hpp"
#include "ThreeAxisSensorMsg.hpp"
#include "Roll_PVProvider.hpp"
#include "Pitch_PVProvider.hpp"
#include "Yaw_PVProvider.hpp"
#include "logger.hpp"

class GyroOrientationObserver : public Roll_PVProvider,
								public Pitch_PVProvider,
								public Yaw_PVProvider,
								public msg_receiver
{
public:

	GyroOrientationObserver();
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

	AttitudeMsg m_filtered_attitude;
	HeadingMsg m_filtered_heading;
	Vector3D<float> gyro_data;

	void loopInternal();
};