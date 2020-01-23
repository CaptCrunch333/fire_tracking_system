#pragma once
#include "Block.hpp"
#include "ControllerMessage.hpp"
#include "FloatMsg.hpp"
#include "common_types.hpp"

class Controller : public Block{

private:
    block_id _name; 
    block_type _type;

public:

    void switchIn(DataMessage*);
    DataMessage* switchOut();
    block_id getID();
    block_type getType();
    virtual controller_type getControllerType() = 0;
    virtual DataMessage* receive_msg_internal(DataMessage*) = 0;
    virtual void receive_msg_data(DataMessage* t_msg) = 0;
    
    Controller(block_id t_name, block_type t_type);
    ~Controller();
};