#pragma once
#include "DataMessage.hpp"

class FloatMessage : public DataMessage{

private:
    msg_type _type;
    float _data;

public:

    msg_type getType();
    const int getSize();
    float getData();
    void setFloatMessage(float);

    FloatMessage();
    ~FloatMessage();
};