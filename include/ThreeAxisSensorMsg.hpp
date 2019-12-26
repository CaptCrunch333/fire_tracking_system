#pragma once 

#include "Vector3D.hpp"
#include "DataMessage.hpp"
#include "common_types.hpp"

class ThreeAxisSensorMsg : public DataMessage
{
public:

	msg_type getType();
	const int getSize();

	Vector3D<float> data; 
};