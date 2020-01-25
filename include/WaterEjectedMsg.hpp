#pragma once
#include "DataMessage.hpp"

class WaterEjectedMsg : public DataMessage
{
public:

    WaterEjectedMsg();
    ~WaterEjectedMsg();

    msg_type getType();
    const int getSize();

    float data;
    
};