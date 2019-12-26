#pragma once
#include "PVProvider.hpp"

class PitchRate_PVProvider : public PVProvider
{

public:

    Vector3D<float> getProcessVariable();
    virtual Vector3D<float> getBodyRate() = 0;
    PitchRate_PVProvider();
    ~PitchRate_PVProvider();

};