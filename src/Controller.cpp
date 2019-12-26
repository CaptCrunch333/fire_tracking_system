#include "Controller.hpp"

Controller::Controller(block_id t_name, block_type t_type) {
    _name = t_name;
    _type = t_type;
}

Controller::~Controller() {

}

void Controller::switchIn(DataMessage* data){
    //this->emit_message(data);
    //TODO implement
}

DataMessage* Controller::switchOut(){
    //TODO implement
    DataMessage* msg;
    return msg;
} 

block_type Controller::getType(){
    return _type;
}

block_id Controller::getID(){
    return _name;
}
