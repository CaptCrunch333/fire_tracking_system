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

#pragma once

#include "ThermalCam.hpp"
#include "TempRange.hpp"
#include "HeatCenterProvider.hpp"

class Lepton3_5 : public ThermalCam
{
public: 
	Lepton3_5();
	CamSpecs getSpecs();
	float getPixelScale(void);
	float getAngPerx(void);
	float getAngPery(void);
	void getDimensions(int &tmp_x, int &tmp_y);
	void receive_msg_data(DataMessage*);

private:
	static ThermalImageMsg m_image;
	ros::NodeHandle _n;
    ros::Subscriber temp_range;
	CamSpecs _lepton_specs;
	int _cam_mode;
	float _pixel_scale;
	
	static void dyn_temp(const water_ext::temp_range::ConstPtr& msg);
};