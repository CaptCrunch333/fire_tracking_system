#pragma once
#include "Provider.hpp"
#include "AccelerationMsg.hpp"

class AccelerationProvider : public Provider{

public:

    virtual AccelerationMsg getAcceleration() = 0;
      
    
    AccelerationProvider();
    ~AccelerationProvider();
};