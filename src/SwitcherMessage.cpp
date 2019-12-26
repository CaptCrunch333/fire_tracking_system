#include "SwitcherMessage.hpp"

SwitcherMessage::SwitcherMessage(){

}

SwitcherMessage::~SwitcherMessage() {

}

void SwitcherMessage::setSwitcherMessage(switcher_type t_source, switcher_type t_destination, internal_switcher_type t_internal_type, Vector3D<float> t_vector3Ddata) {
    _type = msg_type::switcher;
    _internal_type = t_internal_type;
    _v3D_data = t_vector3Ddata;
    _source = t_source;
    _destination = t_destination;
}

void SwitcherMessage::setSwitcherMessage(switcher_type t_source, switcher_type t_destination, internal_switcher_type t_internal_type, float t_float_data){
    _type = msg_type::switcher;
    _internal_type = t_internal_type;
    _float_data = t_float_data;
    _source = t_source;
    _destination = t_destination;
}

void SwitcherMessage::setSwitcherMessage(Vector3D<float> t_vector3Ddata){
    _type = msg_type::switcher;
    _v3D_data = t_vector3Ddata;
}

void SwitcherMessage::setSwitcherMessage(float t_data){
    _type = msg_type::switcher;
    _float_data = t_data;
}

msg_type SwitcherMessage::getType(){
    return _type;
}

const int SwitcherMessage::getSize()
{
    return sizeof(this);
}

internal_switcher_type SwitcherMessage::getInternalType(){
    return _internal_type;
}

Vector3D<float> SwitcherMessage::getVector3DData(){
    return _v3D_data;
}

float SwitcherMessage::getFloatData(){
    return _float_data;
}

switcher_type SwitcherMessage::getSource(){
    return _source;
}

switcher_type SwitcherMessage:: getDestination(){
    return _destination;
}