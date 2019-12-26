#pragma once
#include "Vector3D.hpp"
#include "Provider.hpp"
#include "Quaternion.hpp"
#include "AttitudeMsg.hpp"

class AttitudeProvider : public Provider{

    public:

        virtual AttitudeMsg getAttitude() = 0;
        
        AttitudeProvider();
        ~AttitudeProvider();
    
    private:

        AttitudeMsg m_att_msg;
};