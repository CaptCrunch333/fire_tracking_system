#pragma once
#include "Block.hpp"
#include <math.h>
#include "ControllerMessage.hpp"
#include "PID_values.hpp"
#include "FloatMsg.hpp"
#include "Controller.hpp"
#include "SwitcherMessage.hpp"
#include "FloatMsg.hpp"
#include "ResetControllerMsg.hpp"
#include "logger.hpp"

class PIDController : public Controller{

    private:
        controller_type _controller_type;
        FloatMsg m_output_msg;
        block_id _name;
        //Chehadeh's code
        PID_parameters parameters;
        bool i_term, d_term, dd_term; //Comparing against booleans is faster
        float _dt;
	    bool en_pv_derivation = true, en_anti_windup = false;
        void set_internal_sw(PID_parameters pid_para_x);
        //---------------
        
    public:
        //Chehadeh's code
        float prev_err = 0, prev2_err = 0, prev_pv_rate = 0, accum_u = 0, accum_I = 0;
        void initialize(void*);
        float pid_inc(float err, float pv_first,float pv_second=-1);
        float pid_direct(float err, float pv_first,float pv_second=-1);
        void set_I_term(float);
        //---------------
        void receive_msg_data(DataMessage* t_msg); 
        void reset();
        DataMessage* receive_msg_internal(DataMessage*);
        controller_type getControllerType();
        //TODO Send a message to Switcher
        //TODO Receive a message from Switcher

        PIDController(block_id t_name, block_type t_type);
        ~PIDController();
};
