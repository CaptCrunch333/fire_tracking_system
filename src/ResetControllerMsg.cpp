#include "ResetControllerMsg.hpp"

ResetControllerMsg::ResetControllerMsg() {

}

ResetControllerMsg::~ResetControllerMsg() {

}

msg_type ResetControllerMsg::getType(){
    return _type;
}

const int ResetControllerMsg::getSize()
{
    return sizeof(this);
}

int ResetControllerMsg::getData(){
    return _data;
}

void ResetControllerMsg::setResetControllerMessage(int t_data){
    _type = msg_type::RESETCONTROLLER;
    _data = t_data;
}