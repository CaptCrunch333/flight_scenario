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
    ros::ServiceClient _update_controller_pid_client;
    ros::ServiceClient _update_controller_mrft_client;
    ros::ServiceClient _update_controller_sm_client;


public:
    enum receiving_channels {pid, mrft, sm};
    void receiveMsgData(DataMessage* t_msg);
    void receiveMsgData(DataMessage* t_msg, int t_channel);

    ROSUnit_UpdateController(ros::NodeHandle&);
    ~ROSUnit_UpdateController();
};