#include "AP_Mission.h"

#include "Copter.h"

#if MODE_AUTO_ENABLED == ENABLED

void AP_Mission_Copter::complete() {
    copter.mode_auto.exit_mission();
    AP_Mission::complete();
}

// void AP_Mission::start_do_cmd() {
//     return mode_auto.start_command(cmd);
// }
// bool AP_Mission::verify_do_cmd() {
//     return mode_auto.verify_command_callback(cmd);
// }

// bool AP_Mission::start_nav_cmd() {
//     return mode_auto.start_command(cmd);
// }
// bool AP_Mission::verify_nav_cmd(const AP_Mission::Mission_Command& cmd) {
//     return mode_auto.verify_command_callback(cmd);
// }

#endif
