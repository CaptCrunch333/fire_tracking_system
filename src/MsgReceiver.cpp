// Version: 1.0
// Author: Pedro Henrique R. P. B. Silva
// Date: 19 November 2019

#include "MsgReceiver.hpp"


void msg_receiver::assign_message_type_ID(msg_type i){
    this->_msg_type=i;
    //// std::cout<<"assign_message_type_ID"<<std::endl;
}

msg_type msg_receiver::get_message_type_ID(){
    //// std::cout<<"get_message_type_ID"<<std::endl;
    return this->_msg_type;
}
msg_receiver::msg_receiver(){
    // std::cout<<"msg_receiver"<<std::endl;
}