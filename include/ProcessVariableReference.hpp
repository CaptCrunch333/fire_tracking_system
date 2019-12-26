#pragma once
#include "Reference.hpp"
#include "FloatMessage.hpp"
#include "Vector3D.hpp"
#include "Vector3DMessage.hpp"

class ProcessVariableReference : public Reference{

private:
    reference_type _reference_type;
    float _reference_value;
    Vector3DMessage m_error_msg;

public:
    reference_type getReferenceType();
      
    void setReferenceValue(float);
    DataMessage* receive_msg_internal(DataMessage*);
    ProcessVariableReference(block_id t_name, block_type t_type);
    ~ProcessVariableReference();
};