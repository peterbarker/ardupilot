#include "AP_Mission.h"

#include "Copter.h"

void AP_Mission_Copter::complete() {
    copter.mode_auto.exit_mission();
    AP_Mission::complete();
}
