#include "ComplementaryFilterSettings.hpp"

ComplementaryFilterSettings::ComplementaryFilterSettings(bool t_use_wrap, float tmp_dt, float tmp_alpha)
{
	d_t = tmp_dt;
	alpha = tmp_alpha;
	beta = 1.f - alpha;
	use_wrap = t_use_wrap;
}

ComplementaryFilterSettings::ComplementaryFilterSettings()
{
	
}

float ComplementaryFilterSettings::getAlpha(void)
{
	return alpha;
}

float ComplementaryFilterSettings::getBeta(void)
{
	return beta;
}

filter_type ComplementaryFilterSettings::getType(void)
{
	return filter_type::COMPLEMENTARY;
}