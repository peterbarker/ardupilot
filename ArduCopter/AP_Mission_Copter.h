#pragma once

#include <AP_Mission/AP_Mission.h>
#include "config.h"

#if MODE_AUTO_ENABLED == ENABLED

class AP_Mission_Copter : public AP_Mission {
public:

    using AP_Mission::AP_Mission;

    void complete();

    void start_command(const AP_Mission::Mission_Command& cmd) override;
    bool verify_command(const AP_Mission::Mission_Command& cmd) override;
};

#endif
