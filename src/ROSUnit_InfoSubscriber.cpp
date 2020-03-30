#include "ROSUnit_InfoSubscriber.hpp"
ROSUnit_InfoSubscriber* ROSUnit_InfoSubscriber::_instance_ptr = NULL;
InfoMsg ROSUnit_InfoSubscriber::info_msg;

ROSUnit_InfoSubscriber::ROSUnit_InfoSubscriber(ros::NodeHandle& t_main_handler) : ROSUnit(t_main_handler)  {

    _sub_info = t_main_handler.subscribe("info", 2, callbackInfo);
    _instance_ptr = this;

}

ROSUnit_InfoSubscriber::~ROSUnit_InfoSubscriber() {

}

void ROSUnit_InfoSubscriber::callbackInfo(const flight_controller::Info& msg){

    //info_msg.number_of_waypoints = msg.number_of_waypoints;
    info_msg.armed = msg.armed;
    
    _instance_ptr->emit_message((DataMessage*) &info_msg); 

}

void ROSUnit_InfoSubscriber::receive_msg_data(DataMessage* t_msg){

}