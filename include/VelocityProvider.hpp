#pragma once
#include "Provider.hpp"
#include "VelocityMsg.hpp"

class VelocityProvider : public Provider{

public:

    virtual VelocityMsg getVelocity() = 0;
      

    VelocityProvider();
    ~VelocityProvider();
};