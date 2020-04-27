#pragma once
#include "FlightElement.hpp"
#include "MessageToBlock.hpp"
#include "RestrictedNormRefSettingsMsg.hpp"
#include "MessageToBlock.hpp"
#include "PositionMsg.hpp"
#include "common_srv/Vector3DMessage.hpp"

class SetRestNormSettings : public FlightElement{

private:
	bool _enabled, _delete;
    float _max_norm;

public:
    RestrictedNormRefSettingsMsg _settings_msg;

    void perform();

    void receiveMsgData(DataMessage* t_msg);
    
    SetRestNormSettings(bool, bool, float);
    ~SetRestNormSettings();
};
