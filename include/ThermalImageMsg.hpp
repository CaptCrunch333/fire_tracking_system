#pragma once

#include <opencv2/core/mat.hpp>
#include "DataMessage.hpp"
#include "CamSpecs.hpp"

class ThermalImageMsg : public DataMessage
{
public: 

    msg_type getType();
    const int getSize();

    cv::Mat image;
    float temp_max, temp_min;
    CamSpecs cam_spec;
};

