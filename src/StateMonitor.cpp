#include "StateMonitor.hpp"

StateMonitor::StateMonitor() {
    _input_port_0 = new InputPort(ports_id::IP_0_INFO, this);
    _input_port_1 = new InputPort(ports_id::IP_1_POSITION, this);
    _input_port_2 = new InputPort(ports_id::IP_2_ERROR, this);
    _output_port_0 = new OutputPort(ports_id::OP_0, this);
    _ports = {_input_port_0, _input_port_1, _input_port_2, _output_port_0};
}

StateMonitor::~StateMonitor() {

}

void StateMonitor::perform() {
    while(1){
        //TODO remove TAKING_OFF, considering it's being done by Outdoor navigation
        if(MainMissionStateManager.getMissionState() != uav_control_states::TAKING_OFF &&
            MainMissionStateManager.getMissionState() != uav_control_states::LANDING){

            if(_error > 0){
                MainMissionStateManager.updateMissionState(uav_control_states::ERROR);
            }else if(_number_of_waypoints > 0 && _armed){
                MainMissionStateManager.updateMissionState(uav_control_states::FOLLOWING_TRAJECTORY);
            }else if(_number_of_waypoints == 0 && _armed && _altitude > MIN_ALT_FOR_HOVERING){
                MainMissionStateManager.updateMissionState(uav_control_states::HOVERING);
            }else if(!_armed){
                MainMissionStateManager.updateMissionState(uav_control_states::LANDED);
            }
        }
        if(MainMissionStateManager.getMissionState() != old_state){
            int_msg.data = (int)(MainMissionStateManager.getMissionState());
            _output_port_0->receiveMsgData((DataMessage*)&int_msg);
            old_state = MainMissionStateManager.getMissionState();
        }
        sleep(0.1);
    }
}

void StateMonitor::process(DataMessage* t_msg, Port* t_port) {
    if(_input_port_0->getID() == IP_0_INFO){
        InfoMsg* info_msg = (InfoMsg*)t_msg;
        _number_of_waypoints = info_msg->number_of_waypoints;
        _armed = info_msg->armed;
    }
    else if(_input_port_0->getID() == IP_1_POSITION){
        PositionMsg* pos_msg = (PositionMsg*)t_msg;
        _altitude = pos_msg->z;
    }
    else if(_input_port_0->getID() == IP_2_ERROR){
        ErrorMsg* error_msg = (ErrorMsg*)t_msg;
        if(error_msg->error){
            _error++;
        }
    }
}