#pragma once

#include "math.h"
#include "Provider.hpp"
#include "ImageConverter.hpp"
#include "ThermalImageMsg.hpp"
#include "NozzleOrientationMsg.hpp"
#include "MsgReceiver.hpp"
#include "logger.hpp"

class  HeatCenterProvider : public Provider, public msg_receiver
{
public:

	HeatCenterProvider();
	void setCutOffTemperature(float);
	void receive_msg_data(DataMessage*);

private:

	NozzleOrientationMsg orientation;
	float _t_cutoff;
	void calcFireCenter(cv::Mat, CamSpecs cam);
	void calcFireAngles(int x,int y, CamSpecs cam);
	int calcThreshold(float, float);
	void getHeatCenter(ThermalImageMsg*);
};