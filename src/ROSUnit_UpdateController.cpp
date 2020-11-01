#include "ROSUnit_UpdateController.hpp"

ROSUnit_UpdateController::ROSUnit_UpdateController(ros::NodeHandle& t_main_handler) : ROSUnit(t_main_handler){
    _input_port_0 = new InputPort(ports_id::IP_0_PID, this);
    _input_port_1 = new InputPort(ports_id::IP_1_MRFT, this);
    _input_port_2 = new InputPort(ports_id::IP_2_SM, this);
    _ports = {_input_port_0, _input_port_1, _input_port_2};
    _update_controller_pid_client = t_main_handler.serviceClient<flight_controller::Update_Controller_PID>("update_controller/pid");
    _update_controller_mrft_client = t_main_handler.serviceClient<flight_controller::Update_Controller_MRFT>("update_controller/mrft");
    _update_controller_bb_client = t_main_handler.serviceClient<flight_controller::Update_Controller_SM>("update_controller/sm");

}

ROSUnit_UpdateController::~ROSUnit_UpdateController() {

}

void ROSUnit_UpdateController::process(DataMessage* t_msg, Port* t_port) {

    if(t_port->getID() == ports_id::IP_0_PID) {
        ControllerMessage* _update_msg = (ControllerMessage*)t_msg;
    
        flight_controller::Update_Controller_PID srv;
        srv.request.controller_parameters.id = (int)(_update_msg->getID());
        srv.request.controller_parameters.pid_kp = _update_msg->getPIDParam().kp;
        srv.request.controller_parameters.pid_ki = _update_msg->getPIDParam().ki;
        srv.request.controller_parameters.pid_kd = _update_msg->getPIDParam().kd;
        srv.request.controller_parameters.pid_kdd = _update_msg->getPIDParam().kdd;
        srv.request.controller_parameters.pid_anti_windup = _update_msg->getPIDParam().anti_windup;
        srv.request.controller_parameters.pid_en_pv_derivation = _update_msg->getPIDParam().en_pv_derivation;
        bool success = _update_controller_pid_client.call(srv);
        if (success)
        {
            ROS_INFO("CONTROLLER UPDATED. id: %d", srv.request.controller_parameters.id);
        }
        else 
        {
            ROS_ERROR("Failed to call service /update_controller");
        }
    }
    else if(t_port->getID() == ports_id::IP_1_MRFT) {

        ControllerMessage* _update_msg = (ControllerMessage*)t_msg;
    
        flight_controller::Update_Controller_MRFT srv;
        srv.request.controller_parameters.id = (int)(_update_msg->getID());
        srv.request.controller_parameters.mrft_beta = _update_msg->getMRFTParam().beta;
        srv.request.controller_parameters.mrft_relay_amp = _update_msg->getMRFTParam().relay_amp;
        srv.request.controller_parameters.mrft_bias = _update_msg->getMRFTParam().bias;

        bool success = _update_controller_mrft_client.call(srv);
        if (success)
        {
            ROS_INFO("CONTROLLER UPDATED. id: %d", srv.request.controller_parameters.id);
        }
        else 
        {
            ROS_ERROR("Failed to call service /update_controller");
        }
    }
    else if(t_port->getID() == ports_id::IP_2_SM){
        ControllerMessage* _update_msg = (ControllerMessage*)t_msg;
    
        flight_controller::Update_Controller_SM srv;
        srv.request.controller_parameters.id = (int)(_update_msg->getID());
        srv.request.controller_parameters.bb_alpha1 = _update_msg->getSMParam().alpha1;
        srv.request.controller_parameters.bb_alpha2 = _update_msg->getSMParam().alpha2;
        srv.request.controller_parameters.bb_h1 = _update_msg->getSMParam().h1;
        srv.request.controller_parameters.bb_h2 = _update_msg->getSMParam().h2;
        bool success = _update_controller_bb_client.call(srv);
        if (success)
        {
            ROS_INFO("CONTROLLER UPDATED. id: %d", srv.request.controller_parameters.id);
        }
        else 
        {
            ROS_ERROR("Failed to call service /update_controller");
        }
    }
}