#include "Copter.h"

#if MODE_AUTO_ENABLED == ENABLED

#include "AP_Mission_Copter.h"

void AP_Mission_Copter::complete() {
    AP_Mission::complete();
    copter.mode_auto.exit_mission();
}

void AP_Mission::start_command(const AP_Mission::Mission_Command& cmd) {
    return mode_auto.start_command(cmd);
}
bool AP_Mission::verify_command(const AP_Mission::Mission_Command& cmd) {
    return mode_auto.verify_command_callback(cmd);
}

#endif
