#include "HeatCenterProvider.hpp"

HeatCenterProvider::HeatCenterProvider()
{
	
}

void HeatCenterProvider::setCutOffTemperature(float t_temp)
{
	_t_cutoff = t_temp;
}

int HeatCenterProvider::calcThreshold(float tmp_min, float tmp_max)
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

void HeatCenterProvider::calcFireCenter(cv::Mat tmp, CamSpecs cam)
{
	if(countNonZero(tmp) > 0)
	{
		float* point;
		float my = 0, mry = 0, mx = 0, mrx = 0;
		cv::Mat col_sum, row_sum;
		
		reduce(tmp, col_sum, 0, cv::REDUCE_SUM, CV_32F);
		reduce(tmp, row_sum, 1, cv::REDUCE_SUM, CV_32F);

		point = row_sum.ptr<float>(0);
		for(int i = 0; i<row_sum.rows; i++)
		{
			my = my + point[i];
			mry = mry + point[i]*i;
		}

		point = col_sum.ptr<float>(0);
		for(int i = 0; i<col_sum.cols; i++)
		{
			mx = mx + point[i];
			mrx = mrx + point[i]*i;
		}

		int y = mry/my;
		int x = mrx/mx;
		calcFireAngles(x, y, cam);
	}
	else
	{
		orientation.pitch = 0;
		orientation.yaw = 0;
		orientation.fire_found = false;
	}
}

void HeatCenterProvider::calcFireAngles(int x,int y, CamSpecs cam)
{
	orientation.yaw = (x-(cam.width/2))*cam.ang_per_px * (M_PI/180.f);
	orientation.pitch = ((cam.height/2)-y)*cam.ang_per_py * (M_PI/180.f);
	// orientation.yaw = -1* orientation.yaw;
	// orientation.pitch = -1* orientation.pitch;
	orientation.fire_found = true;
}

void HeatCenterProvider::getHeatCenter(ThermalImageMsg* t_msg)
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
	threshold(t_image, t_image, calcThreshold(t_msg->temp_min, t_msg->temp_max), 255, 3);
	calcFireCenter(t_image, (t_msg->cam_spec));
}

void HeatCenterProvider::receive_msg_data(DataMessage* t_msg)
{
	if(t_msg->getType() == msg_type::THERMALIMAGE)
	{
		this->getHeatCenter((ThermalImageMsg*) t_msg);
		this->emit_message((DataMessage*) &orientation);
	}
}