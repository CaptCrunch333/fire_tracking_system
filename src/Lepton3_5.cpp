//....................................Lepton 3.5..................................................................//
// by Flir placed in pure thermal PCB board.                                                                      //
//  Note about the camera:                                                                                        //
//      1. have three modes of operation: 									  //
//		a. High gain ( from -10C to 140C within 5% accuracy)                                              //
//		b. Low gain (from -10C to 400C at room temperature and to 450C Typically; within 10% accuracy)    //
//	2. Video over SPI                                                                                         //
// 	3. Optimum operation temperature range is from -10C to 80C.                                               //
//	4. sensor technology uncooled VOx microbolometer.                                                         //
// for more specifications refer to the manual at the link below                                                  //
//https://www.flir.com/globalassets/imported-assets/document/flir-lepton-engineering-datasheet.pdf                //
//................................................................................................................//

#include "Lepton3_5.hpp"

ThermalImageMsg Lepton3_5::m_image;

Lepton3_5::Lepton3_5()
{  	
	_lepton_specs.width = 160;
	_lepton_specs.height = 120;
	_lepton_specs.frame_rate = 9; //8.7 Hz
	_lepton_specs.HFOV = 57;
	_lepton_specs.VFOV = 60;
	_lepton_specs.ang_per_px = ((float)_lepton_specs.HFOV)/((float)_lepton_specs.width);
	_lepton_specs.ang_per_py = ((float)_lepton_specs.VFOV)/((float)_lepton_specs.height);
	_lepton_specs.pixel_size = (_lepton_specs.HFOV/_lepton_specs.width)*(_lepton_specs.VFOV/_lepton_specs.height); //in degrees
	_cam_mode = 0;		// the mode represents 2 options [0-> auto grey scale/ 1-> Fixed grey scale]. 
	temp_range = _n.subscribe<positioning_system::temp_range>("/temp_range", 1, dyn_temp);
	m_image.cam_spec = _lepton_specs;
}


void Lepton3_5::receive_msg_data(DataMessage* t_msg)
{
	if(t_msg->getType() == msg_type::THERMALIMAGE)
	{
		ThermalImageMsg* tmp = (ThermalImageMsg*) t_msg;
		tmp->temp_max = m_image.temp_max;
		tmp->temp_min = m_image.temp_min;
		tmp->cam_spec = m_image.cam_spec;
		DataMessage* tee = (DataMessage*) tmp;
		this->emit_message(tee);
	}
}

float Lepton3_5::getAngPerx(void)
{
	return _lepton_specs.ang_per_px;
}

float Lepton3_5::getAngPery(void)
{
	return _lepton_specs.ang_per_py;
}

void Lepton3_5::getDimensions(int &tmp_x, int &tmp_y)
{
	tmp_x = _lepton_specs.width;
	tmp_y = _lepton_specs.height;
}

CamSpecs Lepton3_5::getSpecs()
{
	return _lepton_specs;
}

float Lepton3_5::getPixelScale(void)
{
	return ((float)_lepton_specs.HFOV)/((float)_lepton_specs.width )*(M_PI*1000.f/180.f); 
}

void Lepton3_5::dyn_temp(const positioning_system::temp_range::ConstPtr& msg)
{
	m_image.temp_min = (float) msg->min;
	m_image.temp_max = (float) msg->max;
}

