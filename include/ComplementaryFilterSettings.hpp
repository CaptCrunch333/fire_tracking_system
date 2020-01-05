#pragma once 

#include "FilterSettings.hpp"

class ComplementaryFilterSettings : public FilterSettings
{
	public:

		ComplementaryFilterSettings(bool, float = 0.01, float = 0.98);
		ComplementaryFilterSettings();
		float d_t;
		bool use_wrap;

		filter_type getType();
		float getAlpha();
		float getBeta();

	private:

		float alpha, beta;
};