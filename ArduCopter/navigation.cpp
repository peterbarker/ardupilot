#include "Copter.h"

#include <AP_InternalError/AP_InternalError.h>

// run_nav_updates - top level call for the autopilot
// ensures calculations such as "distance to waypoint" are calculated before autopilot makes decisions
// To-Do - rename and move this function to make it's purpose more clear
void Copter::run_nav_updates(void)
{
    update_super_simple_bearing(false);
}

// distance between vehicle and home in cm
uint32_t Copter::home_distance()
{
    if (position_ok()) {
        _home_distance = current_loc.get_distance(ahrs.get_home()) * 100;
    }
    return _home_distance;
}

// The location of home in relation to the vehicle in centi-degrees
int32_t Copter::home_bearing()
{
    if (position_ok()) {
        _home_bearing = current_loc.get_bearing_to(ahrs.get_home());
    }
    return _home_bearing;
}

int32_t Copter::CurrentLocation::safe_home_relative_alt() const
{
    int32_t relalt;

    if (get_alt_cm(AltFrame::ABOVE_HOME, relalt)) {
        return relalt;
    }

    AP::internalerror().error(AP_InternalError::error_t::home_relalt_no_home);

    if (get_alt_cm(AltFrame::ABOVE_ORIGIN, relalt)) {
        return relalt;
    }

    return 0;
}
