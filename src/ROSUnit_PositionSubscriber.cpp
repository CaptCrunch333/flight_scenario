#include "ROSUnit_PositionSubscriber.hpp"
ROSUnit_PositionSubscriber* ROSUnit_PositionSubscriber::_instance_ptr = NULL;
PositionMsg ROSUnit_PositionSubscriber::position_msg;
Port* ROSUnit_PositionSubscriber::_output_port_0 = new OutputPort(ports_id::OP_0, NULL);

ROSUnit_PositionSubscriber::ROSUnit_PositionSubscriber(ros::NodeHandle& t_main_handler) : ROSUnit(t_main_handler)  {

    _sub_position = t_main_handler.subscribe("/global2inertial/position", 2, callbackPosition);
    _instance_ptr = this;
    _ports = {_output_port_0};
}

ROSUnit_PositionSubscriber::~ROSUnit_PositionSubscriber() {

}

void ROSUnit_PositionSubscriber::callbackPosition(const geometry_msgs::Point& msg){

    position_msg.x = msg.x;
    position_msg.y = msg.y;
    position_msg.z = msg.z;
    _output_port_0->receiveMsgData((DataMessage*) &position_msg);
}