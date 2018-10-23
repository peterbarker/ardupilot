#pragma once

#include <AP_Mission/AP_Mission.h>
#include "config.h"

#if MODE_AUTO_ENABLED == ENABLED

class AP_Mission_Copter : public AP_Mission {
public:

    using AP_Mission::AP_Mission;

    void complete();

    bool start_nav_cmd() override;
    bool nav_cmd_is_complete() const override;

    bool start_do_cmd() override;
    bool do_cmd_is_complete() const override;
};

#endif
