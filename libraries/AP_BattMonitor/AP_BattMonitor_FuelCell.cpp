#include <AP_HAL/AP_HAL.h>
#include <AP_Common/AP_Common.h>
#include <AP_Math/AP_Math.h>
#include "AP_BattMonitor.h"
#include "AP_BattMonitor_FuelCell.h"
#include "AP_FuelCell/AP_FuelCell.h"

extern const AP_HAL::HAL& hal;

/// Constructor
AP_BattMonitor_FuelCell::AP_BattMonitor_FuelCell(AP_BattMonitor &mon,
                                             AP_BattMonitor::BattMonitor_State &mon_state,
                                             AP_BattMonitor_Params &params) :
    AP_BattMonitor_Backend(mon, mon_state, params)
{
    _state.consumed_wh = 0.0;
}

// read - read the voltage and current
void AP_BattMonitor_FuelCell::read()
{
    // get fuel cell lib pointer
    const AP_FuelCell* fuel_cell = AP_FuelCell::get_singleton();
    if (fuel_cell == nullptr) {
        _state.healthy = false;
        return;
    }
    // check that it is enabled
    if (!fuel_cell->enabled()) {
        _state.healthy = false;
        return;
    }
    _state.healthy = fuel_cell->healthy();

    // calculate time since last read
    uint32_t tnow = AP_HAL::micros();
    float dt = tnow - _state.last_time_micros;

    if ((_params._type == AP_BattMonitor_Params::BattMonitor_TYPE_FuelCell_TANK && fuel_cell->healthy()) || 
    (_params._type == AP_BattMonitor_Params::BattMonitor_TYPE_FuelCell_BATTERY && fuel_cell->get_type() == AP_FuelCell::_FUEL_CELL_IE6 && fuel_cell->healthy())) {
        float proportion_remaining = 0.0f;
        if (_params._type == AP_BattMonitor_Params::BattMonitor_TYPE_FuelCell_TANK) {
            // Not available, show a fixed voltage of 1v
            _state.voltage = 1.0;
            proportion_remaining = fuel_cell->get_tank_remain();
        } else {
            // Must be the battery monitor for the 600 800 W unit
            _state.voltage = fuel_cell->get_battery_volt();
            proportion_remaining = fuel_cell->get_battery_remain();
        }

        // Map consumed_mah to consumed percentage
        _state.consumed_mah = (1 - proportion_remaining) * _params._pack_capacity;

        // Map consumed_wh using fixed voltage of 1
        _state.consumed_wh = _state.consumed_mah;

        _state.last_time_micros = tnow;

    } else if (_params._type == AP_BattMonitor_Params::BattMonitor_TYPE_FuelCell_BATTERY && fuel_cell->get_type() == AP_FuelCell::_FUEL_CELL_IE24 && fuel_cell->healthy()) {
        // Inteligent Energy convention is that negative battery power is a draw on the battery positive battery power 
        // is charging the battery.  This is oppose to AP convention hence power is *-1.
        _state.voltage = fuel_cell->get_battery_volt();

        int16_t pwr = -1 * fuel_cell->get_battery_pwr();

        // Calculate current and battery consumed
        if (!is_zero(_state.voltage)) {
            _state.current_amps =  (float)pwr/_state.voltage;

            //0.0000002778f = 1/1E6(us->s) * 1/3600(s->hr) * 1000(A->mA)
            float mah = _state.current_amps * dt * 0.0000002778f;
            _state.consumed_mah += mah;

            _state.consumed_wh = 0.001f * mah * _state.voltage;

        } else {
            _state.current_amps = 0.0f;
        }

        _state.last_time_micros = tnow;

    } else {
        _state.healthy = false;
        _state.voltage = 0;
        _state.consumed_wh = 0;
        _state.consumed_mah = 0;
        _state.current_amps = 0;
    }
}

AP_BattMonitor::BatteryFailsafe AP_BattMonitor_FuelCell::update_failsafes()
{
    AP_BattMonitor::BatteryFailsafe fuel_cell_failsafe = AP_BattMonitor::BatteryFailsafe::BatteryFailsafe_None;

    // only check for fuel cell failsafes on the tank moniter
    // no point in having the same failsafe on two battery's
    if (_params._type == AP_BattMonitor_Params::BattMonitor_TYPE_FuelCell_TANK) {
        // get fuel cell lib pointer
        const AP_FuelCell* fuel_cell = AP_FuelCell::get_singleton();
        if (fuel_cell != nullptr) {
            fuel_cell_failsafe = fuel_cell->update_failsafes();
        }
    }

    return MAX(AP_BattMonitor_Backend::update_failsafes(),fuel_cell_failsafe);
}

// returns false if we fail arming checks, in which case the buffer will be populated with a failure message
bool AP_BattMonitor_FuelCell::arming_checks(char * buffer, size_t buflen) const
{
    // check standard things
    if (!AP_BattMonitor_Backend::arming_checks(buffer, buflen)) {
        return false;
    }

    // if they pass also check fuel cell lib for arming
    const AP_FuelCell* fuel_cell = AP_FuelCell::get_singleton();
    if (fuel_cell == nullptr) {
        char message[17];
        strcpy(message, "Fuel Cell NulPtr");
        strncpy(buffer, message, buflen);
        return false;
    }

    // only arming check the tank moniter
    if (_params._type == AP_BattMonitor_Params::BattMonitor_TYPE_FuelCell_TANK) {
        return fuel_cell->arming_checks(buffer, buflen);
    }

    // if we got this far we are safe to arm
    return true;
}
