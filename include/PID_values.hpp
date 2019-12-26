#pragma once
#include <stdint.h>
#include "DataMessage.hpp"
#include "common_types.hpp"

class PID_parameters : public DataMessage{
//TODO separate PID_Parameters from PID_Parameters_Message
private:
	msg_type _type;

public:
	//TODO add a string NAME
	float kp, ki, kd, kdd, anti_windup, dt = -1;
	block_id id;
	uint8_t en_pv_derivation;

	msg_type getType(){return _type;}
    const int getSize(){return sizeof(this);}
	void setPIDParam(PID_parameters t_param){
		_type = msg_type::UPDATECONTROLLER;
		kp = t_param.kp;
		ki = t_param.ki;
		kd = t_param.kd;
		kdd = t_param.kdd;
		anti_windup = t_param.anti_windup;
		en_pv_derivation = t_param.en_pv_derivation;
		id = t_param.id;
	}
	   
};


class PID_data {

public:

	float err, pv_first, pv_second;
	   
};