#include "Switcher.hpp"

Switcher::Switcher(std::string t_name, switcher_type t_type, control_system t_parent) {
    _type = t_type;
    _name = t_name;
    _parent = t_parent;
    _active_block = nullptr;
}

Switcher::~Switcher() {

}

void Switcher::addBlock(Block* b){
    
    _blocks.push_back(b);
}


switcher_type Switcher::getType(){
    return _type;
}

void Switcher::switchBlock(Block* from, Block* to){
    
    DataMessage* msg;    
    from->switchOut();
    to->switchIn(msg);
}

Block* Switcher::getActiveBlock(){
    return _active_block;
}

std::string Switcher::getID(){
    return _name;
}

//TODO add reference block so the error can be calculated
void Switcher::receive_msg_data(DataMessage* t_msg){
    
    if(t_msg->getType() == msg_type::control_system){

        ControlSystemMessage* control_system_msg = (ControlSystemMessage*)t_msg;
        Block* block_to_add = control_system_msg->getBlockToAdd();
        
        //Considering the message is sent to all the Switchers, this checks if the block being altered belongs to that switcher
        //(4)
        if(control_system_msg->getControlSystemMsgType() == control_system_msg_type::switch_in_out
            && std::find(_blocks.begin(), _blocks.end(), block_to_add) != _blocks.end()){

            Block* block_to_remove = control_system_msg->getBlockToRemove();                
            //Check if blocks to switch are of the same type, because the message sent by the Control System will go to both swithcers (Controller
            // and Reference)
            if(block_to_remove->getType() == block_to_add->getType()){
                block_to_add->switchIn(block_to_remove->switchOut());
                _active_block = block_to_add;          
            }
        //(5)
        } else if (control_system_msg->getControlSystemMsgType() == control_system_msg_type::add_block
                    && static_cast<int>(this->getType()) == static_cast<int>(block_to_add->getType())){ //TODO Refactor
                
            Block* block_to_add = control_system_msg->getBlockToAdd();
            if(this->_blocks.empty()){
                _active_block = block_to_add;
            }
            this->addBlock(block_to_add);
        //(6)
        } else if (control_system_msg->getControlSystemMsgType() == control_system_msg_type::change_PID_settings){ //TODO Refactor to change_Controller_sett
            //TODO send to all the blocks, so we can update before switching
            if(_active_block->getType() == block_type::controller){
                Controller* controller_block = (Controller*)_active_block; //TODO refactor
                if(controller_block->getControllerType() == controller_type::pid){
                    PIDController* pid_block = (PIDController*)controller_block;
                    pid_block->initialize(control_system_msg->getPIDSettings());
                } //TODO else if MRFT
            } 
        //(7)
        } else if(control_system_msg->getControlSystemMsgType() == control_system_msg_type::provider_data){
            
            Vector3D<float> data_provided = control_system_msg->getV3DData();
            
            if(_active_block->getType() == block_type::reference){
                Reference* reference_block = (Reference*)_active_block;

                if(reference_block->getReferenceType() == reference_type::process_variable_ref){
                    ProcessVariableReference* pv_ref_block = (ProcessVariableReference*)reference_block;
                    //TODO remove Vector3DMessage 
                    m_process_variable.setVector3DMessage(data_provided);

                    DataMessage* output_from_reference = pv_ref_block->receive_msg_internal((DataMessage*) &m_process_variable);

                    m_process_variable.setVector3DMessage(((Vector3DMessage*)output_from_reference)->getData());

                    m_reference_msg.setSwitcherMessage(m_process_variable.getData());
                    
                    this->emit_message((DataMessage*) &m_reference_msg);

                }//TODO add other references as else if

            }
        
        //(9)
        } else if(control_system_msg->getControlSystemMsgType() == control_system_msg_type::SETREFERENCE){
            float reference = control_system_msg->getData();
            if(_active_block->getType() == block_type::reference){
                Reference* _reference_block = (Reference*)_active_block;
                _reference_block->setReferenceValue(reference);
            }
        }

    //(8)  //Controller Switcher
    }else if(t_msg->getType() == msg_type::switcher){

        SwitcherMessage* switcher_msg = (SwitcherMessage*)t_msg;
                      
        DataMessage* output = _active_block->receive_msg_internal((DataMessage*)switcher_msg);

        FloatMsg* data = (FloatMsg*)output;
        m_out_switcher_msg.setSwitcherMessage(data->data);
        
        this->emit_message((DataMessage*) &m_out_switcher_msg);
    }

}