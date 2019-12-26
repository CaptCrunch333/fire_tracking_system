#pragma once

class CamSpecs
{
public:
	int width;
	int height;
	int frame_rate;
	int HFOV;
	int VFOV;
	int pixel_size; //Degrees
	float ang_per_px;
	float ang_per_py;
};