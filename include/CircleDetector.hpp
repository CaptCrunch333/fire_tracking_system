#pragma once

#include "math.h"
#include "Provider.hpp"
#include "ImageConverter.hpp"
#include "ThermalImageMsg.hpp"
#include "NozzleOrientationMsg.hpp"
#include "MsgReceiver.hpp"
#include "logger.hpp"
#include <stdio.h>


class  CircleDetector : public Provider, public msg_receiver
{
public:

	CircleDetector();
	~CircleDetector();
	void setCutOffTemperature(float);
	void receive_msg_data(DataMessage*);

private:

	NozzleOrientationMsg orientation;
	float _t_cutoff;

	//circle vector
	std::vector<cv::Vec3f> circles;

	//init
	bool initialized = false;
	float ring_center_x = 0;
	float ring_center_y = 0;
	float ring_center_radius = 15;

	//filtering params
	float ring_center_radius_var = 100;
	float ring_measured_radius_var = 100;
	float ring_match_threshold = 0.7; //from 0 to 1

	//circle detection params
	float dp =1.5;
	float minDist = 30;
	float param1 = 80;
	float param2 = 20;
	float minRadius = 3;
	float maxRadius = 20;

	//thresholds
	float lower_threshold = 0;
	float upper_threshold = 180;

	//functions
	void getFireCircle(cv::Mat);
	void checkSanity();
	void ringMatchScores(std::vector<float>&, int&);

	void calcFireCenter(cv::Mat, CamSpecs cam);
	void calcFireAngles(int x,int y, CamSpecs cam);
	int calcThreshold(float, float);
	void getHeatCenter(ThermalImageMsg*);
};