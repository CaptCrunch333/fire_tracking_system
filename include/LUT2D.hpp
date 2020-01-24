#pragma once 

#include <string>
#include <fstream>
#include "logger.hpp"

class LUT2D
{
	public:
		void setLUT(std::string);
		float getVal(float);
	
	private:
		float m_LUT[100][2];
		int m_length;
};