#include "Copter.h"

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
    Location home;
    if (position_ok() && ahrs.get_home(home)) {
        _home_distance = current_loc.get_distance(home) * 100;
    }
    return _home_distance;
}

// The location of home in relation to the vehicle in centi-degrees
int32_t Copter::home_bearing()
{
    Location home;
    if (position_ok() && ahrs.get_home(home)) {
        _home_bearing = current_loc.get_bearing_to(home);
    }
    return _home_bearing;
}
