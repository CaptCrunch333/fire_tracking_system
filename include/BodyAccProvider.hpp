#pragma once 

#include "Vector3D.hpp"

class BodyAccProvider
{
public:

	virtual Vector3D<float> getBodyAcceleration() = 0;

};