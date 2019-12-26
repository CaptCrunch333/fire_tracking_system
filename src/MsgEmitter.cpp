// Version: 1.0
// Author: Pedro Henrique R. P. B. Silva
// Date: 19 November 2019

#include "MsgEmitter.hpp"

void msg_emitter::add_callback_msg_receiver(msg_receiver* _callback_msg_receiver){
    _list_of_msg_receivers.push_back(_callback_msg_receiver);
    //// std::cout<<"msg_emitter::add_callback_msg_receiver "<<_list_of_msg_receivers.size()<<std::endl;
}

void msg_emitter::emit_message(DataMessage* t_msg){
    std::list<msg_receiver*>::iterator it;
    //// std::cout<<"msg_emitter::emit_message, msg_type: "<<(int)_msg_type<<std::endl;
    for (it = _list_of_msg_receivers.begin(); it != _list_of_msg_receivers.end(); ++it){
        //// std::cout<<"msg_emitter::emit_message, msg size: "<<len<<std::endl;
        (*it)->receive_msg_data(t_msg);
    }
    //// std::cout<<"msg_emitter::emit_message "<<_list_of_msg_receivers.size()<<std::endl;
}
msg_emitter::msg_emitter(){
}

