#pragma once
#include "FlightElement.hpp"
#include "InfoMsg.hpp"
#include "PositionMsg.hpp"
#include "ErrorMsg.hpp"
#include "common_srv/IntegerMsg.hpp"
#include "MissionStateManager.hpp"
#include "internal_states.hpp"

const float MIN_ALT_FOR_HOVERING = 0.2;


class StateMonitor : public FlightElement{

private:
    Port* _input_port_0;
    Port* _input_port_1;
    Port* _input_port_2;
    Port* _output_port_0;
    int _number_of_waypoints, _error = 0;
    bool _armed;
    float _altitude;
    IntegerMsg int_msg;
    uav_control_states old_state = uav_control_states::HOVERING;
public:
    enum ports_id {IP_0_INFO, IP_1_POSITION, IP_2_ERROR, OP_0};
    void perform();
    void process(DataMessage* t_msg, Port* t_port);

    StateMonitor();
    ~StateMonitor();
};