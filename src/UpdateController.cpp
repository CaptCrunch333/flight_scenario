#include "UpdateController.hpp"

void UpdateController::perform(){
    ControllerMessage _pid_parameters_message;
    //TODO refactor, remove id from PIDData and send separately
    _pid_parameters_message.setPIDParam(this->pid_data);
    _pid_parameters_message.setMRFTParam(this->mrft_data);
    _pid_parameters_message.setSMParam(this->bb_data);

    _output_port_0->receiveMsgData((DataMessage*)&_pid_parameters_message);
}

UpdateController::UpdateController(){
    _output_port_0 = new OutputPort(ports_id::OP_0, this);
    _ports = {_output_port_0};
}
UpdateController::~UpdateController(){

}
