#pragma once
#include "Vector3DMessage.hpp"
#include "common_types.hpp"
#include "AttitudeProvider.hpp"
#include "PositioningProvider.hpp"
#include "HeadingProvider.hpp"
#include "AccelerationProvider.hpp"

class PVProvider : public Provider, public Vector3DMessage{

public:

    virtual Vector3D<float> getProcessVariable() = 0;

    PVProvider() {};
};