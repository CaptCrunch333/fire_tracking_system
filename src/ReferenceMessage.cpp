#include "ReferenceMessage.hpp"

ReferenceMessage::ReferenceMessage(Vector3D<float> t_v3d){
    _type = msg_type::reference;
    _data = t_v3d.x;
}

ReferenceMessage::ReferenceMessage(float t_float){
    _type = msg_type::reference;
    _data = t_float;
}

ReferenceMessage::ReferenceMessage(){
}

ReferenceMessage::~ReferenceMessage() {

}

void ReferenceMessage::setReferenceMessage(Vector3D<float> t_v3d){
    _type = msg_type::reference;
    _data = t_v3d.x;
}

void ReferenceMessage::setReferenceMessage(float t_float){
    _type = msg_type::reference;
    _data = t_float;
}

msg_type ReferenceMessage::getType(){
    return _type;
}

const int ReferenceMessage::getSize()
{
    return sizeof(this);
}

float ReferenceMessage::getData(){
    return _data;
}