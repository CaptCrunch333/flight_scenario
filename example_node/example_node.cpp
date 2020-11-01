#include "ros/ros.h"
#include <iostream>
#include "common_srv/ROSUnit.hpp"
#include "logger.hpp"
#include "std_logger.hpp"
#include "FlightElement.hpp"
#include "Wait.hpp"
#include "WaitForCondition.hpp"
#include "Arm.hpp"
#include "FlightPipeline.hpp"
#include "SimplePlaneCondition.hpp"
#include "Disarm.hpp"
#include "FlightScenario.hpp"
#include "UpdateController.hpp"
#include "ResetController.hpp"
#include "SwitchTrigger.hpp"
#include "ROSUnit_Arm.hpp"
#include "ROSUnit_UpdateController.hpp"
#include "ROSUnit_PositionSubscriber.hpp"
#include "ROSUnit_ResetController.hpp"
#include "ROSUnit_OrientationSubscriber.hpp"
#include "ROSUnit_FlightCommand.hpp"
#include "FlightCommand.hpp"
#include "ROSUnit_InfoSubscriber.hpp"
#include "ChangeInternalState.hpp"
#include "InternalSystemStateCondition.hpp"
#include "StateMonitor.hpp"
#include "common_srv/ROSUnit_Factory.hpp"
#include "ROSUnit_RestNormSettingsClnt.hpp"
#include "SetRestNormSettings.hpp"
#include "SetHeightOffset.hpp"
#include "SetRelativeWaypoint.hpp"
#include "SetAbsoluteWaypoint.hpp"
#include "ROSUnit_ControlOutputSubscriber.hpp"
#include "DNNConfirmationCondition.hpp"
#include "SetCameraStatus.hpp"
#include <math.h>

#define TESTING
#define SMALL_HEXA

int main(int argc, char** argv) {
//     Logger::assignLogger(new StdLogger());

    //****************ROS Units********************
    ros::init(argc, argv, "flight_scenario");
    ros::NodeHandle nh;

    ROSUnit* ros_arm_srv = new ROSUnit_Arm(nh);
    ROSUnit* ros_updt_ctr = new ROSUnit_UpdateController(nh);
    ROSUnit* ros_pos_sub = new ROSUnit_PositionSubscriber(nh);
    //ROSUnit* ros_ori_sub = new ROSUnit_OrientationSubscriber(nh);
    ROSUnit* ros_rst_ctr = new ROSUnit_ResetController(nh);
    ROSUnit* ros_flight_command = new ROSUnit_FlightCommand(nh);
    ROSUnit* ros_info_sub = new ROSUnit_InfoSubscriber(nh);
    ROSUnit* ros_restnorm_settings = new ROSUnit_RestNormSettingsClnt(nh);
    

    ROSUnit_Factory ROSUnit_Factory_main{nh};
	ROSUnit* ros_set_path_clnt = ROSUnit_Factory_main.CreateROSUnit(ROSUnit_tx_rx_type::Client,
                                                                    ROSUnit_msg_type::ROSUnit_Poses,
                                                                    "uav_control/set_path");
	// ROSUnit* ros_set_hover_point_srv = ROSUnit_Factory_main.CreateROSUnit(ROSUnit_tx_rx_type::Server, 
    //                                                                 ROSUnit_msg_type::ROSUnit_Point,
    //                                                                 "uav_control/set_hover_point");
    // ROSUnit* ros_set_mission_state_srv = ROSUnit_Factory_main.CreateROSUnit(ROSUnit_tx_rx_type::Server,
    //                                                                         ROSUnit_msg_type::ROSUnit_Int,
    //                                                                         "uav_control/set_mission_state");
    // ROSUnit* ros_uav_attitude_srv = ROSUnit_Factory_main.CreateROSUnit(ROSUnit_tx_rx_type::Server,
    //                                                                 ROSUnit_msg_type::ROSUnit_Float,
    //                                                                 "uav_control/uav_attitude");
    ROSUnit* ros_updt_uav_control_state_clnt = ROSUnit_Factory_main.CreateROSUnit(ROSUnit_tx_rx_type::Client,
                                                                                ROSUnit_msg_type::ROSUnit_Int,
                                                                                "ex_bldg_fire_mm/update_uav_control_state");
    ROSUnit* ros_set_height_offset = ROSUnit_Factory_main.CreateROSUnit(ROSUnit_tx_rx_type::Client,
                                                                    ROSUnit_msg_type::ROSUnit_Float,
                                                                    "set_height_offset"); 
    // ROSUnit* rosunit_x_provider = ROSUnit_Factory_main.CreateROSUnit(ROSUnit_tx_rx_type::Subscriber, 
    //                                                                 ROSUnit_msg_type::ROSUnit_Point,
    //                                                                 "/providers/x");
    // ROSUnit* rosunit_y_provider = ROSUnit_Factory_main.CreateROSUnit(ROSUnit_tx_rx_type::Subscriber, 
    //                                                                 ROSUnit_msg_type::ROSUnit_Point,
    //                                                                 "/providers/y");
    // ROSUnit* rosunit_z_provider = ROSUnit_Factory_main.CreateROSUnit(ROSUnit_tx_rx_type::Subscriber, 
    //                                                                 ROSUnit_msg_type::ROSUnit_Point,
    //                                                                 "/providers/z");
    // ROSUnit* rosunit_roll_provider = ROSUnit_Factory_main.CreateROSUnit(ROSUnit_tx_rx_type::Subscriber, 
    //                                                                 ROSUnit_msg_type::ROSUnit_Point,
    //                                                                 "/providers/roll");
    // ROSUnit* rosunit_pitch_provider = ROSUnit_Factory_main.CreateROSUnit(ROSUnit_tx_rx_type::Subscriber, 
    //                                                                 ROSUnit_msg_type::ROSUnit_Point,
    //                                                                 "/providers/pitch");
    ROSUnit* rosunit_yaw_provider = ROSUnit_Factory_main.CreateROSUnit(ROSUnit_tx_rx_type::Subscriber, 
                                                                    ROSUnit_msg_type::ROSUnit_Point,
                                                                    "/providers/yaw");
    // ROSUnit* rosunit_yaw_rate_provider = ROSUnit_Factory_main.CreateROSUnit(ROSUnit_tx_rx_type::Subscriber, 
    //                                                                 ROSUnit_msg_type::ROSUnit_Point,
    //                                                                 "/providers/yaw_rate");

//     //*****************Flight Elements*************

    FlightElement* update_controller_pid_x = new UpdateController();
    FlightElement* update_controller_pid_y = new UpdateController();
    FlightElement* update_controller_pid_z = new UpdateController();
    FlightElement* update_controller_pid_roll = new UpdateController();
    FlightElement* update_controller_pid_pitch = new UpdateController();
    FlightElement* update_controller_pid_yaw = new UpdateController();
    FlightElement* update_controller_pid_yaw_rate = new UpdateController();

    FlightElement* reset_z = new ResetController();

    FlightElement* arm_motors = new Arm();
    FlightElement* disarm_motors = new Disarm();

    FlightElement* flight_command = new FlightCommand();

    FlightElement* state_monitor = new StateMonitor();

    FlightElement* set_settings = new SetRestNormSettings(true, false, 0.5); 

    FlightElement* land_set_settings = new SetRestNormSettings(true, false, 0.15);
    FlightElement* waypoint_set_settings = new SetRestNormSettings(true, false, 0.40); 

    FlightElement* set_height_offset = new SetHeightOffset();
    FlightElement* initial_pose_waypoint = new SetRelativeWaypoint(0., 0., 0., 0.);
    
    #ifdef TESTING
    FlightElement* takeoff_relative_waypoint = new SetRelativeWaypoint(0., 0., 1.0, 0.);
    #endif

    //FlightElement* absolute_zero_Z_relative_waypoint = new SetRelativeWaypoint(0., 0., -10, 0.); 
    FlightElement* absolute_origin_1m_height = new SetAbsoluteWaypoint(0, 0, 1, 0);
    FlightElement* absolute_waypoint_square_1 = new SetAbsoluteWaypoint(1.5, 0., 1.0, 0.);
    FlightElement* absolute_waypoint_square_2 = new SetAbsoluteWaypoint(1.5, 1.5, 1.0, 0.);
    FlightElement* absolute_waypoint_square_3 = new SetAbsoluteWaypoint(-1.5, 1.5, 1.0, 0.);
    FlightElement* absolute_waypoint_square_4 = new SetAbsoluteWaypoint(-1.5, -1.5, 1.0, 0.);
    FlightElement* absolute_waypoint_square_5 = new SetAbsoluteWaypoint(1.5, -1.5, 1.0, 0.);
    FlightElement* absolute_waypoint_square_6 = new SetAbsoluteWaypoint(1.5, 0.0, 1.0, 0.);
    FlightElement* absolute_waypoint_square_7 = new SetAbsoluteWaypoint(0.0, 0.0, 1.0, 0.);
    FlightElement* land_relative_waypoint = new SetRelativeWaypoint(0., 0., -2., 0.);

//     //******************Connections***************

    update_controller_pid_x->getPorts()[UpdateController::ports_id::OP_0]->connect((ros_updt_ctr)->getPorts()[ROSUnit_UpdateController::ports_id::IP_0_PID]);
    update_controller_pid_y->getPorts()[UpdateController::ports_id::OP_0]->connect((ros_updt_ctr)->getPorts()[ROSUnit_UpdateController::ports_id::IP_0_PID]);
    update_controller_pid_z->getPorts()[UpdateController::ports_id::OP_0]->connect((ros_updt_ctr)->getPorts()[ROSUnit_UpdateController::ports_id::IP_0_PID]);
    update_controller_pid_roll->getPorts()[UpdateController::ports_id::OP_0]->connect((ros_updt_ctr)->getPorts()[ROSUnit_UpdateController::ports_id::IP_0_PID]);
    update_controller_pid_pitch->getPorts()[UpdateController::ports_id::OP_0]->connect((ros_updt_ctr)->getPorts()[ROSUnit_UpdateController::ports_id::IP_0_PID]);
    update_controller_pid_yaw->getPorts()[UpdateController::ports_id::OP_0]->connect((ros_updt_ctr)->getPorts()[ROSUnit_UpdateController::ports_id::IP_0_PID]);
    update_controller_pid_yaw_rate->getPorts()[UpdateController::ports_id::OP_0]->connect((ros_updt_ctr)->getPorts()[ROSUnit_UpdateController::ports_id::IP_0_PID]);

    ros_pos_sub->getPorts()[ROSUnit_PositionSubscriber::ports_id::OP_0]->connect(initial_pose_waypoint->getPorts()[SetRelativeWaypoint::ports_id::IP_0]);
    rosunit_yaw_provider->getPorts()[ROSUnit_PointSub::ports_id::OP_5]->connect(initial_pose_waypoint->getPorts()[SetRelativeWaypoint::ports_id::IP_1]);
    
    ros_pos_sub->getPorts()[ROSUnit_PositionSubscriber::ports_id::OP_0]->connect(takeoff_relative_waypoint->getPorts()[SetRelativeWaypoint::ports_id::IP_0]);
    rosunit_yaw_provider->getPorts()[ROSUnit_PointSub::ports_id::OP_5]->connect(takeoff_relative_waypoint->getPorts()[SetRelativeWaypoint::ports_id::IP_1]);
    //rosunit_yaw_provider->connect(absolute_zero_Z_relative_waypoint);
    //ros_pos_sub->connect(absolute_zero_Z_relative_waypoint);
    ros_pos_sub->getPorts()[ROSUnit_PositionSubscriber::ports_id::OP_0]->connect(land_relative_waypoint->getPorts()[SetRelativeWaypoint::ports_id::IP_0]);
    rosunit_yaw_provider->getPorts()[ROSUnit_PointSub::ports_id::OP_5]->connect(land_relative_waypoint->getPorts()[SetRelativeWaypoint::ports_id::IP_1]);

    ros_pos_sub->getPorts()[ROSUnit_PositionSubscriber::ports_id::OP_0]->connect(set_height_offset->getPorts()[SetHeightOffset::ports_id::IP_0]);

    reset_z->getPorts()[ResetController::ports_id::OP_0]->connect(ros_rst_ctr->getPorts()[ROSUnit_ResetController::ports_id::IP_0]);

    arm_motors->getPorts()[Arm::ports_id::OP_0]->connect(ros_arm_srv->getPorts()[ROSUnit_Arm::ports_id::IP_0]);
    disarm_motors->getPorts()[Disarm::ports_id::OP_0]->connect(ros_arm_srv->getPorts()[ROSUnit_Arm::ports_id::IP_0]);

    ros_flight_command->getPorts()[ROSUnit_FlightCommand::ports_id::OP_0]->connect(flight_command->getPorts()[FlightCommand::ports_id::IP_0]);

    ros_info_sub->getPorts()[ROSUnit_InfoSubscriber::ports_id::OP_0]->connect(state_monitor->getPorts()[StateMonitor::ports_id::IP_0_INFO]);
    ros_pos_sub->getPorts()[ROSUnit_PositionSubscriber::ports_id::OP_0]->connect(state_monitor->getPorts()[StateMonitor::ports_id::IP_1_POSITION]);

    state_monitor->getPorts()[StateMonitor::ports_id::OP_0]->connect(ros_updt_uav_control_state_clnt->getPorts()[ROSUnit_SetIntClnt::IP_0]);

    set_settings->getPorts()[SetRestNormSettings::ports_id::OP_0]->connect(ros_restnorm_settings->getPorts()[ROSUnit_RestNormSettingsClnt::ports_id::IP_0]);
    land_set_settings->getPorts()[SetRestNormSettings::ports_id::OP_0]->connect(ros_restnorm_settings->getPorts()[ROSUnit_RestNormSettingsClnt::ports_id::IP_0]);
    waypoint_set_settings->getPorts()[SetRestNormSettings::ports_id::OP_0]->connect(ros_restnorm_settings->getPorts()[ROSUnit_RestNormSettingsClnt::ports_id::IP_0]);
    
    set_height_offset->getPorts()[SetHeightOffset::ports_id::OP_0]->connect(ros_set_height_offset->getPorts()[ROSUnit_SetFloatClnt::ports_id::IP_0]);
    initial_pose_waypoint->getPorts()[SetRelativeWaypoint::ports_id::OP_0]->connect(ros_set_path_clnt->getPorts()[ROSUnit_SetPosesClnt::ports_id::IP_0]);
    takeoff_relative_waypoint->getPorts()[SetRelativeWaypoint::ports_id::OP_0]->connect(ros_set_path_clnt->getPorts()[ROSUnit_SetPosesClnt::ports_id::IP_0]);
    //absolute_zero_Z_relative_waypoint->connect(ros_set_path_clnt);
    absolute_origin_1m_height->getPorts()[SetAbsoluteWaypoint::ports_id::OP_0]->connect(ros_set_path_clnt->getPorts()[ROSUnit_SetPosesClnt::ports_id::IP_0]);
    absolute_waypoint_square_1->getPorts()[SetAbsoluteWaypoint::ports_id::OP_0]->connect(ros_set_path_clnt->getPorts()[ROSUnit_SetPosesClnt::ports_id::IP_0]);
    absolute_waypoint_square_2->getPorts()[SetAbsoluteWaypoint::ports_id::OP_0]->connect(ros_set_path_clnt->getPorts()[ROSUnit_SetPosesClnt::ports_id::IP_0]);
    absolute_waypoint_square_3->getPorts()[SetAbsoluteWaypoint::ports_id::OP_0]->connect(ros_set_path_clnt->getPorts()[ROSUnit_SetPosesClnt::ports_id::IP_0]);
    absolute_waypoint_square_4->getPorts()[SetAbsoluteWaypoint::ports_id::OP_0]->connect(ros_set_path_clnt->getPorts()[ROSUnit_SetPosesClnt::ports_id::IP_0]);
    absolute_waypoint_square_5->getPorts()[SetAbsoluteWaypoint::ports_id::OP_0]->connect(ros_set_path_clnt->getPorts()[ROSUnit_SetPosesClnt::ports_id::IP_0]);
    absolute_waypoint_square_6->getPorts()[SetAbsoluteWaypoint::ports_id::OP_0]->connect(ros_set_path_clnt->getPorts()[ROSUnit_SetPosesClnt::ports_id::IP_0]);
    absolute_waypoint_square_7->getPorts()[SetAbsoluteWaypoint::ports_id::OP_0]->connect(ros_set_path_clnt->getPorts()[ROSUnit_SetPosesClnt::ports_id::IP_0]);
    land_relative_waypoint->getPorts()[SetRelativeWaypoint::ports_id::OP_0]->connect(ros_set_path_clnt->getPorts()[ROSUnit_SetPosesClnt::ports_id::IP_0]);

//     //*************Setting Flight Elements*************
    #ifdef SMALL_HEXA

    ((UpdateController*)update_controller_pid_x)->pid_data.kp = 0.696435; //0.51639 * 0.8;
    ((UpdateController*)update_controller_pid_x)->pid_data.ki = 0.0;
    ((UpdateController*)update_controller_pid_x)->pid_data.kd = 0.375166; //0.21192 * 0.8;
    ((UpdateController*)update_controller_pid_x)->pid_data.kdd = 0.0;
    ((UpdateController*)update_controller_pid_x)->pid_data.anti_windup = 0;
    ((UpdateController*)update_controller_pid_x)->pid_data.en_pv_derivation = 1;
    ((UpdateController*)update_controller_pid_x)->pid_data.dt = 1./120;
    ((UpdateController*)update_controller_pid_x)->pid_data.id = block_id::PID_X;

    ((UpdateController*)update_controller_pid_y)->pid_data.kp = 0.568331;// 0.51639 * 0.8;
    ((UpdateController*)update_controller_pid_y)->pid_data.ki = 0.0;
    ((UpdateController*)update_controller_pid_y)->pid_data.kd =  0.306157;// * 0.8;
    ((UpdateController*)update_controller_pid_y)->pid_data.kdd = 0.0;
    ((UpdateController*)update_controller_pid_y)->pid_data.anti_windup = 0;
    ((UpdateController*)update_controller_pid_y)->pid_data.en_pv_derivation = 1;
    ((UpdateController*)update_controller_pid_y)->pid_data.dt = 1./120;
    ((UpdateController*)update_controller_pid_y)->pid_data.id = block_id::PID_Y;

    #ifdef TESTING
    ((UpdateController*)update_controller_pid_z)->pid_data.kp = 0.730936; 
    ((UpdateController*)update_controller_pid_z)->pid_data.ki = 0.0980*2; 
    ((UpdateController*)update_controller_pid_z)->pid_data.kd = 0.190225; 
    #endif

    ((UpdateController*)update_controller_pid_z)->pid_data.kdd = 0.0;
    ((UpdateController*)update_controller_pid_z)->pid_data.anti_windup = 0;
    ((UpdateController*)update_controller_pid_z)->pid_data.en_pv_derivation = 1;
    ((UpdateController*)update_controller_pid_z)->pid_data.dt = 1./120;
    ((UpdateController*)update_controller_pid_z)->pid_data.id = block_id::PID_Z;

    ((UpdateController*)update_controller_pid_roll)->pid_data.kp = 0.2121; //0.172195; //0.3302; //0.286708; //0.225 * 0.8; 
    ((UpdateController*)update_controller_pid_roll)->pid_data.ki = 0.0;
    ((UpdateController*)update_controller_pid_roll)->pid_data.kd = 0.0489; //0.042464; //0.0931; //0.056559; //0.04 * 0.8;
    ((UpdateController*)update_controller_pid_roll)->pid_data.kdd = 0.0;
    ((UpdateController*)update_controller_pid_roll)->pid_data.anti_windup = 0;
    ((UpdateController*)update_controller_pid_roll)->pid_data.en_pv_derivation = 1;
    ((UpdateController*)update_controller_pid_roll)->pid_data.id = block_id::PID_ROLL;

    ((UpdateController*)update_controller_pid_pitch)->pid_data.kp = 0.2506;// 0.3360; //0.2811;//0.275252; //0.225 * 0.8; 
    ((UpdateController*)update_controller_pid_pitch)->pid_data.ki = 0.0;
    ((UpdateController*)update_controller_pid_pitch)->pid_data.kd =  0.0578;//0.0684; //0.053100; //0.0868;// 0.051266; //0.04 * 0.8; 
    ((UpdateController*)update_controller_pid_pitch)->pid_data.kdd = 0.0;
    ((UpdateController*)update_controller_pid_pitch)->pid_data.anti_windup = 0;
    ((UpdateController*)update_controller_pid_pitch)->pid_data.en_pv_derivation = 1;
    ((UpdateController*)update_controller_pid_pitch)->pid_data.id = block_id::PID_PITCH;

    ((UpdateController*)update_controller_pid_yaw)->pid_data.kp = 1.6 * 2;
    ((UpdateController*)update_controller_pid_yaw)->pid_data.ki = 0.0;
    ((UpdateController*)update_controller_pid_yaw)->pid_data.kd = 0.0;
    ((UpdateController*)update_controller_pid_yaw)->pid_data.kdd = 0.0;
    ((UpdateController*)update_controller_pid_yaw)->pid_data.anti_windup = 0;
    ((UpdateController*)update_controller_pid_yaw)->pid_data.en_pv_derivation = 1;
    ((UpdateController*)update_controller_pid_yaw)->pid_data.id = block_id::PID_YAW;

    ((UpdateController*)update_controller_pid_yaw_rate)->pid_data.kp = 0.16 * 2;
    ((UpdateController*)update_controller_pid_yaw_rate)->pid_data.ki = 0.0;
    ((UpdateController*)update_controller_pid_yaw_rate)->pid_data.kd = 0.0;
    ((UpdateController*)update_controller_pid_yaw_rate)->pid_data.kdd = 0.0;
    ((UpdateController*)update_controller_pid_yaw_rate)->pid_data.anti_windup = 0;
    ((UpdateController*)update_controller_pid_yaw_rate)->pid_data.en_pv_derivation = 1;
    ((UpdateController*)update_controller_pid_yaw_rate)->pid_data.id = block_id::PID_YAW_RATE;

    #endif

    ((ResetController*)reset_z)->target_block = block_id::PID_Z;

    Wait wait_1s;
    wait_1s.wait_time_ms=1000;

    Wait wait_100ms;
    wait_100ms.wait_time_ms=100;
  
    #ifdef TESTING
    FlightPipeline testing_pipeline;

    testing_pipeline.addElement((FlightElement*)&wait_1s);
    testing_pipeline.addElement((FlightElement*)update_controller_pid_x);
    testing_pipeline.addElement((FlightElement*)update_controller_pid_y);
    testing_pipeline.addElement((FlightElement*)update_controller_pid_z);
    testing_pipeline.addElement((FlightElement*)update_controller_pid_roll);
    testing_pipeline.addElement((FlightElement*)update_controller_pid_pitch);
    testing_pipeline.addElement((FlightElement*)update_controller_pid_yaw);
    testing_pipeline.addElement((FlightElement*)update_controller_pid_yaw_rate);

    testing_pipeline.addElement((FlightElement*)set_height_offset);
    testing_pipeline.addElement((FlightElement*)&wait_1s);
    testing_pipeline.addElement((FlightElement*)set_settings);
    testing_pipeline.addElement((FlightElement*)initial_pose_waypoint);
    testing_pipeline.addElement((FlightElement*)flight_command);
    testing_pipeline.addElement((FlightElement*)reset_z);
    testing_pipeline.addElement((FlightElement*)&wait_100ms);
    testing_pipeline.addElement((FlightElement*)arm_motors);
    testing_pipeline.addElement((FlightElement*)flight_command);
    testing_pipeline.addElement((FlightElement*)reset_z);
    testing_pipeline.addElement((FlightElement*)takeoff_relative_waypoint);
    testing_pipeline.addElement((FlightElement*)flight_command);
    testing_pipeline.addElement((FlightElement*)waypoint_set_settings);   
    testing_pipeline.addElement((FlightElement*)&wait_100ms);
    testing_pipeline.addElement((FlightElement*)absolute_origin_1m_height);
    testing_pipeline.addElement((FlightElement*)absolute_waypoint_square_1);
    testing_pipeline.addElement((FlightElement*)absolute_waypoint_square_2);
    testing_pipeline.addElement((FlightElement*)absolute_waypoint_square_3);
    testing_pipeline.addElement((FlightElement*)absolute_waypoint_square_4);
    testing_pipeline.addElement((FlightElement*)absolute_waypoint_square_5);
    testing_pipeline.addElement((FlightElement*)absolute_waypoint_square_6);
    testing_pipeline.addElement((FlightElement*)absolute_waypoint_square_7);
    testing_pipeline.addElement((FlightElement*)flight_command);
    testing_pipeline.addElement((FlightElement*)land_set_settings);   
    testing_pipeline.addElement((FlightElement*)&wait_100ms);
    testing_pipeline.addElement((FlightElement*)land_relative_waypoint);

    #endif

//     Logger::getAssignedLogger()->log("FlightScenario main_scenario",LoggerLevel::Info);
    FlightScenario main_scenario;

    #ifdef TESTING
    main_scenario.AddFlightPipeline(&testing_pipeline);
    #endif

    main_scenario.StartScenario();
//     Logger::getAssignedLogger()->log("Main Done",LoggerLevel::Info);
    
    while(ros::ok){
        ros::spinOnce();
    }
    return 0;
}