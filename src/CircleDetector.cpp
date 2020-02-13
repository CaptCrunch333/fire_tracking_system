#include "CircleDetector.hpp"

CircleDetector::CircleDetector()
{
	
}

void CircleDetector::setCutOffTemperature(float t_temp)
{
	_t_cutoff = t_temp;
}

int CircleDetector::calcThreshold(float tmp_min, float tmp_max)
{
	float _t_threshold;
	_t_threshold = (int) (((255.f)/((float)(tmp_max - tmp_min)))*(_t_cutoff - (float)tmp_min));
	if (_t_threshold < 0)
	{
		_t_threshold = 0;
	}
	else if (_t_threshold > 255)
	{
		_t_threshold = 256;
	}
	return _t_threshold;
}

void CircleDetector::getFireCircle(cv::Mat t_image)
{
	HoughCircles(t_image, circles, CV_HOUGH_GRADIENT, dp, minDist, param1, param2, minRadius, maxRadius);
}

void CircleDetector::checkSanity()
{
	std::vector<float> likelihood_list;
	int max_idx;
	ringMatchScores(likelihood_list, max_idx);
	
	if (!initialized)
	{
		if (likelihood_list.at(max_idx) > ring_match_threshold)
		{
			initialized = true;
			ring_center_x = circles.at(max_idx)[0];
			ring_center_y = circles.at(max_idx)[1];
			ring_center_radius = circles.at(max_idx)[2];
		}
	}
	else if (likelihood_list.at(max_idx) > ring_match_threshold) {
		ring_center_x = circles.at(max_idx)[0];
		ring_center_y = circles.at(max_idx)[1];
		ring_center_radius = circles.at(max_idx)[2];
		ring_center_radius_var = ring_center_radius_var + 10;
		ring_center_radius_var = ring_center_radius_var * ring_measured_radius_var / (ring_center_radius_var + ring_measured_radius_var);
	}
}

void CircleDetector::ringMatchScores(std::vector<float> &t_list, int &t_mx)
{
	t_mx = 0;
	for(int i = 0; i < circles.size(); i++) {
		t_list.push_back(exp(-0.5*sqrt(ring_center_radius - circles.at(i)[2])/ring_center_radius_var));
		if(t_list.at(t_mx) > t_list.at(i)) {
			t_mx = i;
		}
	}
}

void CircleDetector::calcFireCenter(cv::Mat tmp, CamSpecs cam)
{
	if(countNonZero(tmp) > 0)
	{
		getFireCircle(tmp);
		checkSanity();
		calcFireAngles(ring_center_x, ring_center_y, cam);
	}
	else
	{
		orientation.pitch = 0;
		orientation.yaw = 0;
		orientation.fire_found = false;
	}
}

void CircleDetector::calcFireAngles(int x,int y, CamSpecs cam)
{
	orientation.yaw = (x-(cam.width/2))*cam.ang_per_px * (M_PI/180.f);
	orientation.pitch = ((cam.height/2)-y)*cam.ang_per_py * (M_PI/180.f);
	// orientation.yaw = -1* orientation.yaw;
	// orientation.pitch = -1* orientation.pitch;
	orientation.fire_found = true;
	std::cout <<"x: "<< x <<std::endl;
	std::cout <<"y: "<< y <<std::endl;

}

void CircleDetector::getHeatCenter(ThermalImageMsg* t_msg)
{
	cv::Mat t_image;
	try
	{
	cvtColor(t_msg->image, t_image, CV_BGR2GRAY);
	}
	catch(cv::Exception& e)
	{
		Logger::getAssignedLogger()->log("Image Not Found", LoggerLevel::Error);
	}
	threshold(t_image, t_image, upper_threshold, 255, 0);
	calcFireCenter(t_image, (t_msg->cam_spec));
}

void CircleDetector::receive_msg_data(DataMessage* t_msg)
{
	if(t_msg->getType() == msg_type::THERMALIMAGE)
	{
		this->getHeatCenter((ThermalImageMsg*) t_msg);
		this->emit_message((DataMessage*) &orientation);
	}
}