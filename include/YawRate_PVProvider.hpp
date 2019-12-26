#pragma once
#include "PVProvider.hpp"

class YawRate_PVProvider : public PVProvider
{

public:

    Vector3D<float> getProcessVariable();
    virtual Vector3D<float> getBodyRate() = 0;
    YawRate_PVProvider();
    ~YawRate_PVProvider();
};