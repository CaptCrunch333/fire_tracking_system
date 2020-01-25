#include "WaterEjectedMsg.hpp"

WaterEjectedMsg::WaterEjectedMsg()
{
}

WaterEjectedMsg::~WaterEjectedMsg()
{

}

msg_type WaterEjectedMsg::getType()
{
    return msg_type::WATEREJECTED;
}

const int WaterEjectedMsg::getSize()
{
    return sizeof(WaterEjectedMsg);
}