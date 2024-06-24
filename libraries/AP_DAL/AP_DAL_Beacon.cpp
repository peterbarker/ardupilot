#include "AP_DAL_Beacon.h"

#if AP_BEACON_ENABLED

#include <AP_Logger/AP_Logger.h>
#include "AP_DAL.h"
#include <AP_Vehicle/AP_Vehicle_Type.h>

AP_DAL_Beacon::AP_DAL_Beacon()
{
#if !APM_BUILD_TYPE(APM_BUILD_AP_DAL_Standalone) && !APM_BUILD_TYPE(APM_BUILD_Replay)
    const auto *bcon = AP::beacon();
    _RBCH.count = bcon->count();
    for (uint8_t i=0; i<ARRAY_SIZE(_RBCI); i++) {
        _RBCI[i].instance = i;
    }
#endif
}

void AP_DAL_Beacon::start_frame()
{
    const auto *bcon = AP::beacon();

    const log_RBCH old = _RBCH;
    if (bcon != nullptr) {
        _RBCH.get_vehicle_position_ned_returncode = bcon->get_vehicle_position_ned(_RBCH.vehicle_position_ned, _RBCH.accuracy_estimate);
        Location loc;
        _RBCH.get_origin_returncode = bcon->get_origin(loc);
        _RBCH.enabled = bcon->enabled();
        _RBCH.origin_lat = loc.lat;
        _RBCH.origin_lng = loc.lng;
        _RBCH.origin_alt = loc.alt;
    }
    WRITE_REPLAY_BLOCK_IFCHANGED(RBCH, _RBCH, old);
    if (bcon == nullptr) {
        return;
    }

    for (uint8_t i=0; i<ARRAY_SIZE(_RBCI); i++) {
        log_RBCI &RBCI = _RBCI[i];
        const log_RBCI old_RBCI = RBCI;
        AP_Beacon::BeaconState beacon_state;
        if (bcon->get_beacon_data(i, beacon_state)) {
            RBCI.last_update_ms = beacon_state.last_update_ms();
            RBCI.position = beacon_state.get_position();
            RBCI.distance = beacon_state.get_distance();
            RBCI.healthy = beacon_state.is_healthy();
        }

        WRITE_REPLAY_BLOCK_IFCHANGED(RBCI, RBCI, old_RBCI);
    }
}

bool AP_DAL_Beacon::get_beacon_data(uint8_t i, AP_Beacon::BeaconState& state) const {
    if (i >= ARRAY_SIZE(_RBCI)) {
        return false;
    }
    state.id = 0;  // ERP?
    state.healthy = _RBCI[i].healthy;
    state.distance = _RBCI[i].distance;
    state.distance_update_ms = _RBCI[i].last_update_ms;
    state.position = _RBCI[i].position;

    return true;
}

#endif
