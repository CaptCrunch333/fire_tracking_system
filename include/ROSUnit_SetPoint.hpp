#pragma once

#include "ROSUnit.hpp"
#include "Vector3DMessage.hpp"
#include "common_srv/set_point.h"

class ROSUnit_SetPoint : public ROSUnit{

    public:
        ROSUnit_SetPoint(std::string, ros::NodeHandle&);
        ~ROSUnit_SetPoint();
        void receive_msg_data(DataMessage* t_msg);

    private:

        ros::ServiceClient m_client;
};