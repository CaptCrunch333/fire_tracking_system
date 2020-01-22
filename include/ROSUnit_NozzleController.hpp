#pragma once
#include "ROSUnit.hpp"
#include "std_srvs/Empty.h"

class ROSUnit_NozzleController : public ROSUnit
{
    public:

        ROSUnit_NozzleController(ros::NodeHandle&);
        ~ROSUnit_NozzleController();
        void receive_msg_data(DataMessage* t_msg){};

    protected:

        ros::ServiceServer m_NozzleController_srv;
        static bool service_cb(std_srvs::Empty::Request&, std_srvs::Empty::Response&);
};