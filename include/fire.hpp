#include <opencv2/core/mat.hpp>

class fire
{
public:

	cv::Mat image;
	std::vector<fire> fire_regions;
	bool exists;
	int x_left, x_right, y_up, y_down;
	int x, y;
	int n_regions;
	float f_size;
	float temp;
	float dim_x, dim_y;
};