#pragma once
#include "PVProvider.hpp"
#include "AttitudeProvider.hpp"
#include "BodyRateProvider.hpp"
#include <math.h>
#include "ROSMsg.hpp"

class Roll_PVProvider : public PVProvider, 
                        public AttitudeProvider{

public:

    Vector3D<float> getProcessVariable();
    virtual AttitudeMsg getAttitude() = 0;
    virtual Vector3D<float> getBodyRate() = 0;
    ROSMsg ros_msg;

    Roll_PVProvider();
    ~Roll_PVProvider();
};