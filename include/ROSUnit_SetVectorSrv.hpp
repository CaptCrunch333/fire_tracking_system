#pragma once

#include "ROSUnit.hpp"
#include "VectorMsg.hpp"
#include "common_srv/set_vector.h"

class ROSUnit_SetVectorSrv : public ROSUnit{

    public:
        ROSUnit_SetVectorSrv(std::string, ros::NodeHandle&);
        //ROSUnit_SetVectorSrv(ros::NodeHandle&);
        ~ROSUnit_SetVectorSrv();
        void receive_msg_data(DataMessage* t_msg) {};

    private:

        //static ROSUnit_SetVectorSrv* m_ptr;
        ros::ServiceServer m_server;
        static bool srv_callback(common_srv::set_vector::Request&, common_srv::set_vector::Response&);
};