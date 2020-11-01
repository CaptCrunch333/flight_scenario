#pragma once
#include <flight_controller/Update_Controller_PID.h>
#include <flight_controller/Update_Controller_MRFT.h>
#include <flight_controller/Update_Controller_SM.h>
#include "common_srv/ROSUnit.hpp"
#include "ControllerMessage.hpp"
#include <flight_controller/PID_param.h>
#include <flight_controller/MRFT_param.h>
#include <flight_controller/SM_param.h>

class ROSUnit_UpdateController : public ROSUnit {

private:
    Port* _input_port_0;
    Port* _input_port_1;
    Port* _input_port_2;
    ros::ServiceClient _update_controller_pid_client;
    ros::ServiceClient _update_controller_mrft_client;
    ros::ServiceClient _update_controller_bb_client;


public:
    enum ports_id {IP_0_PID, IP_1_MRFT, IP_2_SM};
    enum receiving_channels {pid, mrft, sm};
    void process(DataMessage* t_msg, Port* t_port); 

    ROSUnit_UpdateController(ros::NodeHandle&);
    ~ROSUnit_UpdateController();
};