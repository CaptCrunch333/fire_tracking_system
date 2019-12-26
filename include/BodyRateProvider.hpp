#pragma once 

#include "Vector3D.hpp"

class BodyRateProvider
{
public:
	virtual Vector3D<float> getBodyRate() = 0;
};