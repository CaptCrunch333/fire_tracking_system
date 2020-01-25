#include "WaterTankLevelMsg.hpp"

msg_type WaterTankLevelMsg::getType()
{
    return msg_type::WATERTANKLEVEL;
}

const int WaterTankLevelMsg::getSize()
{
    return sizeof(WaterTankLevelMsg);
}