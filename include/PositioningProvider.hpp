#pragma once
#include "Vector3D.hpp"
#include "MsgReceiver.hpp"
#include "Block.hpp"
#include "Vector3DMessage.hpp"
#include "Provider.hpp"
#include "PositionMsg.hpp"

class PositioningProvider : public Provider{

    private:
        Vector3D<float> _homePos;
        PositionMsg m_pos_msg;

    public:
        virtual PositionMsg getPosition() = 0;
        void setHomePosition(Vector3D<float>);
        Vector3D<float> getRelativePosition();
        
        PositioningProvider();
        ~PositioningProvider();
        
};