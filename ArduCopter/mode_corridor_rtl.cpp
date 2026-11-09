#include "Copter.h"

#if MODE_CORRIDOR_RTL_ENABLED

/*
 * Corridor RTL: return along the flight corridor graph
 *
 * On entry a route is planned onto the corridor graph (MAV_MISSION_TYPE_CORRIDOR)
 * and along it to the best reachable landing site.  The route is fed
 * to WPNav one point at a time, and the vehicle lands at the site.
 * The route is fixed on entry; a new graph uploaded in flight does not
 * change it.
 */

// Return true if this mode is enabled, used by MAVLink available modes
bool ModeCorridorRTL::enabled() const
{
    return AP::corridor() != nullptr;
}

bool ModeCorridorRTL::init(bool ignore_checks)
{
    const AP_Corridor *corridor = AP::corridor();
    if (corridor == nullptr || corridor->num_nodes() == 0) {
        GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "CorridorRTL: no corridor graph");
        return false;
    }

    Location current_loc;
    if (!AP::ahrs().get_location(current_loc)) {
        return false;
    }

    AP_Corridor::Route new_route;
    if (!corridor->plan_route(current_loc, new_route)) {
        GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "CorridorRTL: no landing site reachable");
        return false;
    }
    delete[] route.points;
    route = new_route;

    static const char *site_names[] { "waypoint", "nest", "divert site", "ditch site" };
    const uint8_t site_type = uint8_t(route.site_type);
    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "CorridorRTL: to %s %u, %u points, joining %.0fm away",
                  site_type < ARRAY_SIZE(site_names) ? site_names[site_type] : "?",
                  unsigned(route.site),
                  unsigned(route.num_points),
                  route.entry_distance_m);

    // initialise waypoint and spline controller
    wp_nav->wp_and_spline_init_m();

    // initialise yaw to obey user parameter
    auto_yaw.set_mode_to_default(true);

    next_point = 0;
    set_submode(SubMode::PATH_FOLLOW);
    set_next_destination();

    return true;
}

void ModeCorridorRTL::exit()
{
    delete[] route.points;
    route = {};
}

void ModeCorridorRTL::set_submode(SubMode submode)
{
    _submode = submode;
    if (submode == SubMode::LAND) {
        land_start();
    }
}

// send the next point of the route to WPNav, and the one after so
// it can fly smoothly past it
void ModeCorridorRTL::set_next_destination()
{
    if (next_point >= route.num_points) {
        return;
    }
    if (!wp_nav->set_wp_destination_loc(route.points[next_point])) {
        // this can only fail on terrain data, which corridor nodes
        // (AMSL) do not use
        INTERNAL_ERROR(AP_InternalError::error_t::flow_of_control);
        set_submode(SubMode::LAND);
        return;
    }
    if (next_point + 1 < route.num_points) {
        IGNORE_RETURN(wp_nav->set_wp_destination_next_loc(route.points[next_point + 1]));
    }
    next_point++;
}

void ModeCorridorRTL::run()
{
    switch (_submode) {
    case SubMode::PATH_FOLLOW:
        path_follow_run();
        break;
    case SubMode::LAND:
        land_run(true);
        break;
    }
}

void ModeCorridorRTL::path_follow_run()
{
    if (wp_nav->reached_wp_destination()) {
        if (next_point >= route.num_points) {
            // over the landing site
            set_submode(SubMode::LAND);
            land_run(true);
            return;
        }
        set_next_destination();
    }

    // update controllers
    motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);
    wp_nav->update_wpnav();
    pos_control->D_update_controller();
    attitude_control->input_thrust_vector_heading(pos_control->get_thrust_vector(), auto_yaw.get_heading());
}

bool ModeCorridorRTL::is_landing() const
{
    return _submode == SubMode::LAND;
}

bool ModeCorridorRTL::get_wp(Location &destination) const
{
    if (_submode != SubMode::PATH_FOLLOW) {
        return false;
    }
    return wp_nav->get_wp_destination_loc(destination);
}

float ModeCorridorRTL::wp_distance_m() const
{
    return wp_nav->get_wp_distance_to_destination_m();
}

float ModeCorridorRTL::wp_bearing_deg() const
{
    return degrees(wp_nav->get_wp_bearing_to_destination_rad());
}

#endif  // MODE_CORRIDOR_RTL_ENABLED
