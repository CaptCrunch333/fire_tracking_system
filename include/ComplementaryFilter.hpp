#pragma once 

#include <cmath>
#include "DataFilter.hpp"
#include "ComplementaryFilterSettings.hpp"

class ComplementaryFilter : public DataFilter
{
public:

	ComplementaryFilter();
	void resetFilter(int);
	void setFilterSettings(FilterSettings*);
	float getFilteredData(float);
	float getFilteredData(float, float);

private:

	float prev_filtered_data = 0;
	ComplementaryFilterSettings filter_settings;

	float value_wrapping_function(float);
	float filter_wrapping_function(float, float);
};