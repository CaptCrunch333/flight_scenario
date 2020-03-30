#pragma once
#include "DataMessage.hpp"

class DoubleMsg : public DataMessage
{
public:

    DoubleMsg();
    ~DoubleMsg();

    msg_type getType();
    const int getSize();

    double data;
    
};