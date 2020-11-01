#include "ROSUnit_RestNormSettingsClnt.hpp"

ROSUnit_RestNormSettingsClnt::ROSUnit_RestNormSettingsClnt(ros::NodeHandle& t_main_handler) : ROSUnit(t_main_handler) {
    _input_port_0 = new InputPort(ports_id::IP_0, this);
    _ports = {_input_port_0};
    _clnt_rest_norm_settings = t_main_handler.serviceClient<flight_controller::Restricted_Norm_Settings>("restricted_norm_settings");
}   

ROSUnit_RestNormSettingsClnt::~ROSUnit_RestNormSettingsClnt() {

}

void ROSUnit_RestNormSettingsClnt::process(DataMessage* t_msg, Port* t_port) {
    
    if(t_port->getID() == ports_id::IP_0)
    {
        RestrictedNormRefSettingsMsg* _settings_msg = (RestrictedNormRefSettingsMsg*)t_msg;
        
        flight_controller::Restricted_Norm_Settings srv;
        srv.request.enabled = _settings_msg->enabled;
        srv.request.delete_existing_waypoints = _settings_msg->delete_existing_waypoints;
        srv.request.max_norm = _settings_msg->getMaxNorm();

        bool success = _clnt_rest_norm_settings.call(srv);

        if (success)
        {
            ROS_INFO("SETTINGS. ENABLED: %d, DELETE_WAYPOINTS: %d, MAX_NORM: %f",srv.request.enabled, srv.request.delete_existing_waypoints,srv.request.max_norm);
        }
        else 
        {
            ROS_ERROR("Failed to call service /switch_block");
        }
    }
}