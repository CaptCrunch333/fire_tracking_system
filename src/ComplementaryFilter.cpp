#include "ComplementaryFilter.hpp"

ComplementaryFilter::ComplementaryFilter()
{

}

void ComplementaryFilter::resetFilter(int t_integrator)
{
	prev_filtered_data = t_integrator;
}

void ComplementaryFilter::setFilterSettings(FilterSettings* t_filter_settings)
{
	filter_settings = *(ComplementaryFilterSettings*)t_filter_settings;
}

float ComplementaryFilter::getFilteredData(float rate)
{
	float filtered_data_bar = prev_filtered_data + rate*filter_settings.d_t;
	if(filter_settings.use_wrap)
	{
		filtered_data_bar = value_wrapping_function(filtered_data_bar);
		prev_filtered_data = filtered_data_bar;
	}
	else
	{
		prev_filtered_data = filtered_data_bar;
	}
	return prev_filtered_data;
}

float ComplementaryFilter::getFilteredData(float val, float rate)
{
	float filtered_data_bar = prev_filtered_data + rate*filter_settings.d_t;
	if(filter_settings.use_wrap)
	{
		filtered_data_bar = value_wrapping_function(filtered_data_bar);
		prev_filtered_data = filter_wrapping_function(val, filtered_data_bar);
	}
	else
	{
		prev_filtered_data = filtered_data_bar*filter_settings.getAlpha() + val*filter_settings.getBeta();
	}
	return prev_filtered_data;
}

float ComplementaryFilter::value_wrapping_function(float inwrap)
{
	if(inwrap < -M_PI)
		return(inwrap + (2.f*M_PI));
	else if (inwrap > M_PI)
		return(inwrap - (2.f*M_PI));
	else
		return(inwrap);
}

float ComplementaryFilter::filter_wrapping_function(float tmp_val, float tmp_val_bar)
{
	if(fabs(tmp_val - tmp_val_bar) > M_PI)
	{
		return(value_wrapping_function(tmp_val_bar*filter_settings.getAlpha() - tmp_val*filter_settings.getBeta()));
	}
	else
	{
		return(value_wrapping_function(tmp_val_bar*filter_settings.getAlpha() + tmp_val*filter_settings.getBeta()));
	}
}