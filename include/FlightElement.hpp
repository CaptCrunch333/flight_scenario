#pragma once

#include "common_srv/Block.hpp"
#include "common_srv/ROSUnit.hpp"
#include "common_srv/InputPort.hpp"
#include "common_srv/OutputPort.hpp"

class FlightElement : public Block {

public:

    virtual void perform() = 0;
    virtual void process(DataMessage* t_msg, Port* t_port) = 0;

    FlightElement();
    ~FlightElement();
};