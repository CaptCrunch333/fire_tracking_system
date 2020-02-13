#pragma once
#include "Block.hpp"
#include <list>
#include "MsgReceiver.hpp"
#include "MsgEmitter.hpp"
#include "ControlSystemMessage.hpp"
#include "SwitcherMessage.hpp"
#include "Vector3DMessage.hpp"
#include "PIDController.hpp"
#include "ProcessVariableReference.hpp"
#include "PositioningProvider.hpp"
#include <algorithm>
#include "AttitudeProvider.hpp"
#include "ReferenceMessage.hpp"
#include "logger.hpp"

class Switcher : public msg_receiver, public msg_emitter{

    private:
        std::list<Block*> _blocks;
        std::list<Block*>::iterator _it;
        switcher_type _type;
        Block* _active_block;
        std::string _name;
        control_system _parent;
        Vector3DMessage m_process_variable;
        SwitcherMessage m_reference_msg;
        SwitcherMessage m_out_switcher_msg;

    public:
        void addBlock(Block* b);
        switcher_type getType();
        Block* getActiveBlock();
        std::string getID();
        void switchBlock(Block* from, Block* to);
        void receive_msg_data(DataMessage* t_msg);
        void loopInternal();
        //TODO Send a message to Block
        //TODO Receive a message from Block
        Switcher(std::string t_name, switcher_type t_type, control_system t_parent);
        ~Switcher();
};