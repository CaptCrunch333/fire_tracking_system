#pragma once 

#include <string>
#include <fstream>

class WaterTankLUT
{
	public:
		void setLUT(std::string);
		int getWaterLevel(int);
	
	private:
		int m_LUT[100][2];
		int m_length;
};