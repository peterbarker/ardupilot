#pragma once

#include <AP_Mission/AP_Mission.h>

class AP_Mission_Copter : public AP_Mission {
public:

    using AP_Mission::AP_Mission;

    void complete();
};
