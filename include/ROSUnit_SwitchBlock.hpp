#pragma once
#include "common_srv/ROSUnit.hpp"
#include "SwitchBlockMsg_FS.hpp"
#include <flight_controller/SwitchBlock.h>

class ROSUnit_SwitchBlock :  public ROSUnit{

    private:
        ros::ServiceClient _switch_client;
        
    public:
        void receiveMsgData(DataMessage* t_msg);  

        ROSUnit_SwitchBlock(ros::NodeHandle&);
        ~ROSUnit_SwitchBlock();
};