/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

// Fuel Cell - support for complex power systems

#include "AP_FuelCell.h"

const AP_Param::GroupInfo AP_FuelCell::var_info[] = {

    // @Param: TYPE
    // @DisplayName: Fuel Cell type
    // @Description: Fuel Cell type
    // @Values: 0:Disabled, 1:Intelligent Energy 650w 800w, 2:Intelligent Energy 2.4kW
    // @User: Standard
    // @RebootRequired: True
    AP_GROUPINFO_FLAGS("TYPE", 1, AP_FuelCell, _type, 0, AP_PARAM_FLAG_ENABLE),

    // @Param: FS_LOW
    // @DisplayName: Failsafes trigger Low action
    // @Description: Failsafes that should trigger battery failsafe Low Action. Bits 32 to 17 are for Intelligent Energy 650w 800w. Bits 16 to 7 are for Intelligent Energy 2.4 kW.
    // @User: Standard
    // @Bitmask: 32: Temp 1 critical, 31: Temp 2 critical, 30: Battery Voltage Critical, 29: Battery Temp Critical, 28: Fan failure, 27: Fan Over Curent, 26: Temp 1 Shutdown, 25: Temp 2 Shutdown, 24: Battery Voltage Shutdown, 23: Battery Temp Shutdown, 22: Start Timeout, 21: Stop Timeout, 20: Start Under Pressue, 19: Tank Under Pressue, 18: Tank Low Pressue, 16:Minor Internal, 15:Reduced Power, 14:SPM_lost, 13:Pressure Low, 12:Battery Low, 11:Pressure Alert, 10:Start Denied, 9:System Critical, 8:Pressure Critical, 7:Battery Critical
    AP_GROUPINFO("FS_LOW", 2, AP_FuelCell, _low_fs_bitmask, 0),

    // @Param: FS_CRIT
    // @DisplayName: Failsafes trigger critical action
    // @Description: Failsafes that should trigger battery failsafe critical Action. Bits 32 to 17 are for Intelligent Energy 650w 800w. Bits 16 to 7 are for Intelligent Energy 2.4 kW.
    // @User: Standard
    // @Bitmask: 32: Temp 1 critical, 31: Temp 2 critical, 30: Battery Voltage Critical, 29: Battery Temp Critical, 28: Fan failure, 27: Fan Over Curent, 26: Temp 1 Shutdown, 25: Temp 2 Shutdown, 24: Battery Voltage Shutdown, 23: Battery Temp Shutdown, 22: Start Timeout, 21: Stop Timeout, 20: Start Under Pressue, 19: Tank Under Pressue, 18: Tank Low Pressue, 16:Minor Internal, 15:Reduced Power, 14:SPM_lost, 13:Pressure Low, 12:Battery Low, 11:Pressure Alert, 10:Start Denied, 9:System Critical, 8:Pressure Critical, 7:Battery Critical
    AP_GROUPINFO("FS_CRIT", 3, AP_FuelCell, _crit_fs_bitmask, 0),

    AP_GROUPEND
};

// Constructor
AP_FuelCell::AP_FuelCell()
{
    AP_Param::setup_object_defaults(this, var_info);
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
    if (_singleton) {
        AP_HAL::panic("Too many fuelcells");
    }
#endif
    _singleton = this;
}

/*
 * Get the AP_FuelCell singleton
 */
AP_FuelCell *AP_FuelCell::get_singleton()
{
    return _singleton;
}

// Return true if fuelcell is enabled
bool AP_FuelCell::enabled() const
{
    return _type != _FUEL_CELL_NONE;
}

// Return fuel cell type
uint8_t AP_FuelCell::get_type() const
{
    return _type;
}

// Initialize the fuelcell object and prepare it for use
void AP_FuelCell::init()
{
    if (_type == _FUEL_CELL_NONE) {
        return;
    }

    _uart = AP::serialmanager().find_serial(AP_SerialManager::SerialProtocol_FuelCell, 0);

    if (_uart != nullptr) {
        _uart->begin(AP::serialmanager().find_baudrate(AP_SerialManager::SerialProtocol_FuelCell, 0));
    } else {
        gcs().send_text(MAV_SEVERITY_INFO, "Fuel Cell: no serial port found");
    }

}

// Update fuelcell, expected to be called at 20hz
void AP_FuelCell::update()
{
    if (_type == 0) {
        return;
    }

    switch (_type) {
        case _FUEL_CELL_IE6:
        case _FUEL_CELL_IE24:
            update_IE();
            break;
    }
}

// update IE backend
void AP_FuelCell::update_IE()
{
    if (_uart == nullptr) {
        return;
    }

    uint32_t now = AP_HAL::millis();

   // read any available data
    int16_t nbytes = _uart->available();
    while (nbytes-- > 0) {
        char c = _uart->read();
        if (decode(c)) {
            // successfully decoded a new reading
            switch(_type) {
                case _FUEL_CELL_IE6:
                    _state.last_time_ms = now;
                    _state.tank_pct = _temp_tank_pct;
                    _state.battery_pct = _temp_battery_pct;
                    _state.state = _temp_state;
                    _state.failsafe = _temp_failsafe;
                    break;

                case _FUEL_CELL_IE24:
                    _state.last_time_ms = now;
                    _state.tank_bar = _temp_tank_bar;
                    _state.battery_volt = _temp_battery_volt;
                    _state.pwr_out = _temp_pwr_out;
                    _state.spm_pwr = _temp_spm_pwr;
                    _state.battery_pwr = _temp_battery_pwr;
                    _state.state = _temp_state;
                    _state.failsafe = _temp_failsafe;
                    break;
            }

            // ensure failsafe bitmask is valid for type
            check_failsafe_type();

            // check if we should notify on any change of fuel cell state
            check_status();
        }
    }

    _state.healthy = (now - _state.last_time_ms) < HEALTHY_TIMEOUT_MS;
}

// add a single character to the buffer and attempt to decode
// returns true if a complete sentence was successfully decoded
bool AP_FuelCell::decode(char c)
{
    // start of a string
    if (c == '<') {
        _sentence_valid = true;
        _term_number = 0;
        _term_offset = 0;
        _in_string = true;
        return false;
    }
    if (!_in_string) {
        return false;
    }

    // end of a string
    if (c == '>') {
        decode_latest_term();
        _in_string = false;
        return _sentence_valid;
    }

    // end of a term in the string
    if (c == ',') {
        decode_latest_term();
        return false;
    }

    // otherwise add the char to the current term
    _term[_term_offset++] = c;

    // we have overrun the expected sentence
    if (_term_offset >TERM_BUFFER) {
        _in_string = false;
    }

    return false;
}

void AP_FuelCell::decode_latest_term()
{
    // null terminate and move onto the next term
    _term[_term_offset] = 0;
    _term_offset = 0;
    _term_number++;

    switch (_type) {
        case _FUEL_CELL_IE6:
            decode_IE6();
            break;
        case _FUEL_CELL_IE24:
            decode_IE24();
            break;
    }
}

// process data string for IE 600 800 W
void AP_FuelCell::decode_IE6()
{
switch (_term_number) {
        case 1:
            _temp_tank_pct = atof(_term);
            // out of range values
            if (_temp_tank_pct > 100.0f || _temp_tank_pct < 0.0f) {
                _sentence_valid = false;
            }
            break;

        case 2:
            _temp_battery_pct = atof(_term);
            // out of range values
            if (_temp_battery_pct > 100.0f || _temp_battery_pct < 0.0f) {
                _sentence_valid = false;
            }
            break;

        case 3:
            _temp_state = strtoul(_term, nullptr, 10);
            break;
        
        case 4:
            _temp_failsafe = strtoul(_term, nullptr, 16);
            //gcs().send_text(MAV_SEVERITY_INFO, "Fuel Cell: %s, %lu",_term,_temp_failsafe);
            break;

        default:
            // if we get any other term number something has gone wrong
            _sentence_valid = false;
            break;
    }
}

// process data string for IE 2.4 W
void AP_FuelCell::decode_IE24()
{
    uint32_t err_code;
switch (_term_number) {
        case 1:
            // float
            _temp_tank_bar = atof(_term);
            break;

        case 2:
            // float
            _temp_battery_volt = atof(_term);
            break;

        case 3:
            // signed int base 10
            _temp_pwr_out = strtol(_term, nullptr, 10);
            break;

        case 4:
            // unsigned int base 10
            _temp_spm_pwr = strtoul(_term, nullptr, 10);
            break;

        case 5:
            // signed int base 10
            _temp_battery_pwr = strtol(_term, nullptr, 10);
            break;

        case 6:
            // unsigned int base 10
            _temp_state = strtoul(_term, nullptr, 10);
            break;

        case 7:
            // unsigned int base 10
            err_code = strtoul(_term, nullptr, 10);

            // convert to bitmask
            if (!err_code_to_failsafe(err_code)) {
                gcs().send_text(MAV_SEVERITY_ALERT, "Fuel Cell: Unknown Error Code");
                _sentence_valid = false;
            }
            break;

        default:
            // if we get any other term number something has gone wrong
            _sentence_valid = false;
            break;
    }
}

// check for failsafes
AP_BattMonitor::BatteryFailsafe AP_FuelCell::update_failsafes() const
{
    if ((_state.failsafe & _crit_fs_bitmask) != 0) {
        return  AP_BattMonitor::BatteryFailsafe::BatteryFailsafe_Critical;
    }

    if ((_state.failsafe & _low_fs_bitmask) != 0) {
        return  AP_BattMonitor::BatteryFailsafe::BatteryFailsafe_Low;
    }

    return AP_BattMonitor::BatteryFailsafe::BatteryFailsafe_None;
}

// check for arming
bool AP_FuelCell::arming_checks(char * buffer, size_t buflen) const
{
    char message[45];

    // refuse arming if not healthy
    if (!healthy()) {
        strcpy(message, "Fuel Cell: driver not healthy");
        strncpy(buffer, message, buflen);
        return false;
    }

    // refuse arming if not in running state
    if (_state.state == _FUEL_CELL_STATE_RUNNING) {
        strcpy(message, "Fuel Cell: Status not running");
        strncpy(buffer, message, buflen);
        return false;
    }

    // refuse for any failsafe
    // IE 600 800 W fail safe codes
    if (_type == _FUEL_CELL_IE6) {
        if ( (_state.failsafe & _FUEL_CELL_FAILSAFE_IE6_STACK_OT1) != 0) {
            strcpy(message, TEMP_1_CRIT);
            strncpy(buffer, message, buflen);
            return false;
        }
        if ( (_state.failsafe & _FUEL_CELL_FAILSAFE_IE6_STACK_OT2) != 0) {
            strcpy(message, TEMP_2_CRIT);
            strncpy(buffer, message, buflen);
            return false;
        }
        if ( (_state.failsafe & _FUEL_CELL_FAILSAFE_IE6_BAT_UV) != 0) {
            strcpy(message, BAT_VOLT_CRIT);
            strncpy(buffer, message, buflen);
            return false;
        }
        if ( (_state.failsafe & _FUEL_CELL_FAILSAFE_IE6_BAT_OT) != 0) {
            strcpy(message, BAT_TEMP_CRIT);
            strncpy(buffer, message, buflen);
            return false;
        }
        if ( (_state.failsafe & _FUEL_CELL_FAILSAFE_IE6_NO_FAN) != 0) {
            strcpy(message, FAN_ERROR);
            strncpy(buffer, message, buflen);
            return false;
        }
        if ( (_state.failsafe & _FUEL_CELL_FAILSAFE_IE6_FAN_OVERRUN) != 0) {
            strcpy(message, FAN_ERROR2);
            strncpy(buffer, message, buflen);
            return false;
        }
        if ( (_state.failsafe & _FUEL_CELL_FAILSAFE_IE6_STACK_OT1_2) != 0) {
            strcpy(message, TEMP_1_SHUT);
            strncpy(buffer, message, buflen);
            return false;
        }
        if ( (_state.failsafe & _FUEL_CELL_FAILSAFE_IE6_STACK_OT2_2) != 0) {
            strcpy(message, TEMP_2_SHUT);
            strncpy(buffer, message, buflen);
            return false;
        }
        if ( (_state.failsafe & _FUEL_CELL_FAILSAFE_IE6_BAT_UV_2) != 0) {
            strcpy(message, BAT_VOLT_SHUT);
            strncpy(buffer, message, buflen);
            return false;
        }
        if ( (_state.failsafe & _FUEL_CELL_FAILSAFE_IE6_BAT_OT_2) != 0) {
            strcpy(message, BAT_TEMP_SHUT);
            strncpy(buffer, message, buflen);
            return false;
        }
        if ( (_state.failsafe & _FUEL_CELL_FAILSAFE_IE6_START_TIMEOUT) != 0) {
            strcpy(message, START_TIMEOUT);
            strncpy(buffer, message, buflen);
            return false;
        }
        if ( (_state.failsafe & _FUEL_CELL_FAILSAFE_IE6_STOP_TIMEOUT) != 0) {
            strcpy(message, STOP_TIMEOUT);
            strncpy(buffer, message, buflen);
            return false;
        }
        if ( (_state.failsafe & _FUEL_CELL_FAILSAFE_IE6_START_UNDER_PRESSURE) != 0) {
            strcpy(message, START_UNDER_PRESS);
            strncpy(buffer, message, buflen);
            return false;
        }
        if ( (_state.failsafe & _FUEL_CELL_FAILSAFE_IE6_TANK_UNDER_PRESSURE) != 0) {
            strcpy(message, TANK_UNDER_PRESS);
            strncpy(buffer, message, buflen);
            return false;
        }
        if ( (_state.failsafe & _FUEL_CELL_FAILSAFE_IE6_TANK_LOW_PRESSURE) != 0) {
            strcpy(message, TANK_LOW_PRESS);
            strncpy(buffer, message, buflen);
            return false;
        }
        if ( (_state.failsafe & _FUEL_CELL_FAILSAFE_IE6_SAFETY_FLAG) != 0) {
            strcpy(message, SAFETY_FLAG);
            strncpy(buffer, message, buflen);
            return false;
        }
    }

    // IE 2.4 kW Failsafe codes
    if (_type == _FUEL_CELL_IE24) {
        if ( (_state.failsafe & _FUEL_CELL_FAILSAFE_IE24_MINOR_INTERNAL) != 0) {
            strcpy(message, MINOR_INTERNAL);
            strncpy(buffer, message, buflen);
            return false;
        }
        if ( (_state.failsafe & _FUEL_CELL_FAILSAFE_IE24_REDUCED_POWER) != 0) {
            strcpy(message, REDUCE_PWR);
            strncpy(buffer, message, buflen);
            return false;
        }
        if ( (_state.failsafe & _FUEL_CELL_FAILSAFE_IE24_SPM_LOST) != 0) {
            strcpy(message, SPM_LOST);
            strncpy(buffer, message, buflen);
            return false;
        }
        if ( (_state.failsafe & _FUEL_CELL_FAILSAFE_IE24_PRESSURE_LOW) != 0) {
            strcpy(message, PRESS_LOW);
            strncpy(buffer, message, buflen);
            return false;
        }
        if ( (_state.failsafe & _FUEL_CELL_FAILSAFE_IE24_BATTERY_LOW) != 0) {
            strcpy(message, BAT_LOW);
            strncpy(buffer, message, buflen);
            return false;
        }
        if ( (_state.failsafe & _FUEL_CELL_FAILSAFE_IE24_PRESSURE_ALERT) != 0) {
            strcpy(message, PRESS_ALERT);
            strncpy(buffer, message, buflen);
            return false;
        }
        if ( (_state.failsafe & _FUEL_CELL_FAILSAFE_IE24_START_DENIED) != 0) {
            strcpy(message, START_DEN);
            strncpy(buffer, message, buflen);
            return false;
        }
        if ( (_state.failsafe & _FUEL_CELL_FAILSAFE_IE24_SYSTEM_CRITICAL) != 0) {
            strcpy(message, SYS_CRIT);
            strncpy(buffer, message, buflen);
            return false;
        }
        if ( (_state.failsafe & _FUEL_CELL_FAILSAFE_IE24_PRESSURE_CRITICAL) != 0) {
            strcpy(message, PRESS_CRIT);
            strncpy(buffer, message, buflen);
            return false;
        }
        if ( (_state.failsafe & _FUEL_CELL_FAILSAFE_IE24_BATTERY_CRITICAL) != 0) {
            strcpy(message, BAT_CRIT);
            strncpy(buffer, message, buflen);
            return false;
        }
    }
    return true;
}

// check for health
bool AP_FuelCell::healthy() const
{
    return _state.healthy;
}

// check for any change in failsafe or status and report
void AP_FuelCell::check_status()
{
    if (_state.state != _state.last_state) {
        switch (_state.state) {
            case _FUEL_CELL_STATE_STARTING:
                gcs().send_text(MAV_SEVERITY_ALERT, STATUS_START);
                break;

            case _FUEL_CELL_STATE_READY:
                gcs().send_text(MAV_SEVERITY_ALERT, STATUS_READY);
                break;

            case _FUEL_CELL_STATE_RUNNING:
                gcs().send_text(MAV_SEVERITY_ALERT, STATUS_RUN);
                break;

            case _FUEL_CELL_STATE_FAULT:
                gcs().send_text(MAV_SEVERITY_ALERT, STATUS_FAULT);
                break;

            case _FUEL_CELL_STATE_BATTERY_ONLY:
                gcs().send_text(MAV_SEVERITY_ALERT, STATUS_BATTERY);
                break;

            default:
                gcs().send_text(MAV_SEVERITY_ALERT, "Fuel Cell: Unknown Status");
                break;

        }
        _state.last_state = _state.state;
    }

    if (_state.failsafe != _state.last_failsafe) {
        // IE 600 800 W Failsafe codes
        if (_type == _FUEL_CELL_IE6) {
            if ((_state.failsafe & _FUEL_CELL_FAILSAFE_IE6_STACK_OT1) != 0 && (_state.last_failsafe & _FUEL_CELL_FAILSAFE_IE6_STACK_OT1) == 0) {
                gcs().send_text(MAV_SEVERITY_ALERT, TEMP_1_CRIT);
            }
            if ((_state.failsafe & _FUEL_CELL_FAILSAFE_IE6_STACK_OT2) != 0 && (_state.last_failsafe & _FUEL_CELL_FAILSAFE_IE6_STACK_OT2) == 0) {
                gcs().send_text(MAV_SEVERITY_ALERT, TEMP_2_CRIT);
            }
            if ((_state.failsafe & _FUEL_CELL_FAILSAFE_IE6_BAT_UV) != 0 && (_state.last_failsafe & _FUEL_CELL_FAILSAFE_IE6_BAT_UV) == 0) {
                gcs().send_text(MAV_SEVERITY_ALERT, BAT_VOLT_CRIT);
            }
            if ((_state.failsafe & _FUEL_CELL_FAILSAFE_IE6_BAT_OT) != 0 && (_state.last_failsafe & _FUEL_CELL_FAILSAFE_IE6_BAT_OT) == 0) {
                gcs().send_text(MAV_SEVERITY_ALERT, BAT_TEMP_CRIT);
            }
            if ((_state.failsafe & _FUEL_CELL_FAILSAFE_IE6_NO_FAN) != 0 && (_state.last_failsafe & _FUEL_CELL_FAILSAFE_IE6_NO_FAN) == 0) {
                gcs().send_text(MAV_SEVERITY_ALERT, FAN_ERROR);
            }
            if ((_state.failsafe & _FUEL_CELL_FAILSAFE_IE6_FAN_OVERRUN) != 0 && (_state.last_failsafe & _FUEL_CELL_FAILSAFE_IE6_FAN_OVERRUN) == 0) {
                gcs().send_text(MAV_SEVERITY_ALERT, FAN_ERROR2);
            }
            if ((_state.failsafe & _FUEL_CELL_FAILSAFE_IE6_STACK_OT1_2) != 0 && (_state.last_failsafe & _FUEL_CELL_FAILSAFE_IE6_STACK_OT1_2) == 0) {
                gcs().send_text(MAV_SEVERITY_ALERT, TEMP_1_SHUT);
            }
            if ((_state.failsafe & _FUEL_CELL_FAILSAFE_IE6_STACK_OT2_2) != 0 && (_state.last_failsafe & _FUEL_CELL_FAILSAFE_IE6_STACK_OT2_2) == 0) {
                gcs().send_text(MAV_SEVERITY_ALERT, TEMP_2_SHUT);
            }
            if ((_state.failsafe & _FUEL_CELL_FAILSAFE_IE6_BAT_UV_2) != 0 && (_state.last_failsafe & _FUEL_CELL_FAILSAFE_IE6_BAT_UV_2) == 0) {
                gcs().send_text(MAV_SEVERITY_ALERT, BAT_VOLT_SHUT);
            }
            if ((_state.failsafe & _FUEL_CELL_FAILSAFE_IE6_BAT_OT_2) != 0 && (_state.last_failsafe & _FUEL_CELL_FAILSAFE_IE6_BAT_OT_2) == 0) {
                gcs().send_text(MAV_SEVERITY_ALERT, BAT_TEMP_SHUT);
            }
            if ((_state.failsafe & _FUEL_CELL_FAILSAFE_IE6_START_TIMEOUT) != 0 && (_state.last_failsafe & _FUEL_CELL_FAILSAFE_IE6_START_TIMEOUT) == 0) {
                gcs().send_text(MAV_SEVERITY_ALERT, START_TIMEOUT);
            }
            if ((_state.failsafe & _FUEL_CELL_FAILSAFE_IE6_STOP_TIMEOUT) != 0 && (_state.last_failsafe & _FUEL_CELL_FAILSAFE_IE6_STOP_TIMEOUT) == 0) {
                gcs().send_text(MAV_SEVERITY_ALERT, STOP_TIMEOUT);
            }
            if ((_state.failsafe & _FUEL_CELL_FAILSAFE_IE6_START_UNDER_PRESSURE) != 0 && (_state.last_failsafe & _FUEL_CELL_FAILSAFE_IE6_START_UNDER_PRESSURE) == 0) {
                gcs().send_text(MAV_SEVERITY_ALERT, START_UNDER_PRESS);
            }
            if ((_state.failsafe & _FUEL_CELL_FAILSAFE_IE6_TANK_UNDER_PRESSURE) != 0 && (_state.last_failsafe & _FUEL_CELL_FAILSAFE_IE6_TANK_UNDER_PRESSURE) == 0) {
                gcs().send_text(MAV_SEVERITY_ALERT, TANK_UNDER_PRESS);
            }
            if ((_state.failsafe & _FUEL_CELL_FAILSAFE_IE6_TANK_LOW_PRESSURE) != 0 && (_state.last_failsafe & _FUEL_CELL_FAILSAFE_IE6_TANK_LOW_PRESSURE) == 0) {
                gcs().send_text(MAV_SEVERITY_ALERT, TANK_LOW_PRESS);
            }
            if ((_state.failsafe & _FUEL_CELL_FAILSAFE_IE6_SAFETY_FLAG) != 0 && (_state.last_failsafe & _FUEL_CELL_FAILSAFE_IE6_SAFETY_FLAG) == 0) {
                gcs().send_text(MAV_SEVERITY_ALERT, SAFETY_FLAG);
            }
        }
        // IE 2.4 kW Failsafe codes
        if (_type == _FUEL_CELL_IE24) {
            if ((_state.failsafe & _FUEL_CELL_FAILSAFE_IE24_MINOR_INTERNAL) != 0 && (_state.last_failsafe & _FUEL_CELL_FAILSAFE_IE24_MINOR_INTERNAL) == 0) {
                gcs().send_text(MAV_SEVERITY_ALERT, MINOR_INTERNAL);
            }
            if ((_state.failsafe & _FUEL_CELL_FAILSAFE_IE24_REDUCED_POWER) != 0 && (_state.last_failsafe & _FUEL_CELL_FAILSAFE_IE24_REDUCED_POWER) == 0) {
                gcs().send_text(MAV_SEVERITY_ALERT, REDUCE_PWR);
            }
            if ((_state.failsafe & _FUEL_CELL_FAILSAFE_IE24_SPM_LOST) != 0 && (_state.last_failsafe & _FUEL_CELL_FAILSAFE_IE24_SPM_LOST) == 0) {
                gcs().send_text(MAV_SEVERITY_ALERT, SPM_LOST);
            }
            if ((_state.failsafe & _FUEL_CELL_FAILSAFE_IE24_PRESSURE_LOW) != 0 && (_state.last_failsafe & _FUEL_CELL_FAILSAFE_IE24_PRESSURE_LOW) == 0) {
                gcs().send_text(MAV_SEVERITY_ALERT, PRESS_LOW);
            }
            if ((_state.failsafe & _FUEL_CELL_FAILSAFE_IE24_BATTERY_LOW) != 0 && (_state.last_failsafe & _FUEL_CELL_FAILSAFE_IE24_BATTERY_LOW) == 0) {
                gcs().send_text(MAV_SEVERITY_ALERT, BAT_LOW);
            }
            if ((_state.failsafe & _FUEL_CELL_FAILSAFE_IE24_PRESSURE_ALERT) != 0 && (_state.last_failsafe & _FUEL_CELL_FAILSAFE_IE24_PRESSURE_ALERT) == 0) {
                gcs().send_text(MAV_SEVERITY_ALERT, PRESS_ALERT);
            }
            if ((_state.failsafe & _FUEL_CELL_FAILSAFE_IE24_START_DENIED) != 0 && (_state.last_failsafe & _FUEL_CELL_FAILSAFE_IE24_START_DENIED) == 0) {
                gcs().send_text(MAV_SEVERITY_ALERT, START_DEN);
            }
            if ((_state.failsafe & _FUEL_CELL_FAILSAFE_IE24_SYSTEM_CRITICAL) != 0 && (_state.last_failsafe & _FUEL_CELL_FAILSAFE_IE24_SYSTEM_CRITICAL) == 0) {
                gcs().send_text(MAV_SEVERITY_ALERT, SYS_CRIT);
            }
            if ((_state.failsafe & _FUEL_CELL_FAILSAFE_IE24_PRESSURE_CRITICAL) != 0 && (_state.last_failsafe & _FUEL_CELL_FAILSAFE_IE24_PRESSURE_CRITICAL) == 0) {
                gcs().send_text(MAV_SEVERITY_ALERT, PRESS_CRIT);
            }
            if ((_state.failsafe & _FUEL_CELL_FAILSAFE_IE24_BATTERY_CRITICAL) != 0 && (_state.last_failsafe & _FUEL_CELL_FAILSAFE_IE24_BATTERY_CRITICAL) == 0) {
                gcs().send_text(MAV_SEVERITY_ALERT, BAT_CRIT);
            }
        }
        _state.last_failsafe = _state.failsafe;
    }
}

// get the tank remaining ratio
float AP_FuelCell::get_tank_remain() const
{
    float out = 0;
    switch (_type) {
        case _FUEL_CELL_IE6:
            out = _state.tank_pct * 0.01;
            break;

        case _FUEL_CELL_IE24:
            // scale tank pressure linearly to a percentage.
            // min = 5 bar, max = 300 bar, 0.003389831f = 1/295.
            out = constrain_float((_state.tank_bar-5)*0.003389831f,0,1);
            break;
    }
    return out;
}

// get the battery remaining ratio
float AP_FuelCell::get_battery_remain() const
{
    float out = 0;
    switch (_type) {
        case _FUEL_CELL_IE6:
            out = _state.battery_pct * 0.01;
            break;

        case _FUEL_CELL_IE24:
            // Not available from 2.4 kW unit
            out = 1;
            break;
    }
    return out;
}

// get the battery pwr
int16_t AP_FuelCell::get_battery_pwr() const
{
    int16_t out = 0;
    switch (_type) {
        case _FUEL_CELL_IE6:
            // Not available from 600 800 W unit
            out = 1;
            break;

        case _FUEL_CELL_IE24:
            out = _state.battery_pwr;
            break;
    }
    return out;
}

// get the battery voltage
float AP_FuelCell::get_battery_volt() const
{
    float out = 0;
    switch (_type) {
        case _FUEL_CELL_IE6:
             // Not available for this unit, report a fixed voltage of 1v
            out = 1.0;
            break;

        case _FUEL_CELL_IE24:
            out = _state.battery_volt;
            break;
    }
    return out;
}


// check selected failsafe bitmask is valid for type
void AP_FuelCell::check_failsafe_type()
{
    uint32_t failsafe_check = 0;

    switch (_type) {
        case _FUEL_CELL_IE6:
            // Bit mask for all error codes used by 600 800 W IE unit
            failsafe_check = _FUEL_CELL_FAILSAFE_IE6_STACK_OT1
                + _FUEL_CELL_FAILSAFE_IE6_STACK_OT2
                + _FUEL_CELL_FAILSAFE_IE6_BAT_UV
                + _FUEL_CELL_FAILSAFE_IE6_BAT_OT
                + _FUEL_CELL_FAILSAFE_IE6_NO_FAN
                + _FUEL_CELL_FAILSAFE_IE6_FAN_OVERRUN
                + _FUEL_CELL_FAILSAFE_IE6_STACK_OT1_2
                + _FUEL_CELL_FAILSAFE_IE6_STACK_OT2_2
                + _FUEL_CELL_FAILSAFE_IE6_BAT_UV_2
                + _FUEL_CELL_FAILSAFE_IE6_BAT_OT_2
                + _FUEL_CELL_FAILSAFE_IE6_START_TIMEOUT
                + _FUEL_CELL_FAILSAFE_IE6_STOP_TIMEOUT
                + _FUEL_CELL_FAILSAFE_IE6_START_UNDER_PRESSURE
                + _FUEL_CELL_FAILSAFE_IE6_TANK_UNDER_PRESSURE
                + _FUEL_CELL_FAILSAFE_IE6_TANK_LOW_PRESSURE
                + _FUEL_CELL_FAILSAFE_IE6_SAFETY_FLAG;
            break;

        case _FUEL_CELL_IE24:
            // Bit mask for all error codes used by 2.4 kW IE unit
            failsafe_check = _FUEL_CELL_FAILSAFE_IE24_MINOR_INTERNAL
                + _FUEL_CELL_FAILSAFE_IE24_REDUCED_POWER
                + _FUEL_CELL_FAILSAFE_IE24_SPM_LOST
                + _FUEL_CELL_FAILSAFE_IE24_PRESSURE_LOW
                + _FUEL_CELL_FAILSAFE_IE24_BATTERY_LOW
                + _FUEL_CELL_FAILSAFE_IE24_PRESSURE_ALERT
                + _FUEL_CELL_FAILSAFE_IE24_START_DENIED
                + _FUEL_CELL_FAILSAFE_IE24_SYSTEM_CRITICAL
                + _FUEL_CELL_FAILSAFE_IE24_PRESSURE_CRITICAL
                + _FUEL_CELL_FAILSAFE_IE24_BATTERY_CRITICAL;
            break;
    }

    // bit masks only valid options for the selected type
    _state.failsafe = _state.failsafe & failsafe_check;
}

// convert the error code received over serial to bit mask
// this conversion is only valid for 2.4 kW version
    bool AP_FuelCell::err_code_to_failsafe(uint32_t err_code)
{
    bool convert_success = true;

    switch (err_code) {
        case _FUEL_CELL_ERR_CODE_NO_ERROR:
            _temp_failsafe = 0;
            break;

        case _FUEL_CELL_ERR_CODE_MINOR_INTERNAL:
            _temp_failsafe = _FUEL_CELL_FAILSAFE_IE24_MINOR_INTERNAL;
            break;

        case _FUEL_CELL_ERR_CODE_REDUCED_POWER:
            _temp_failsafe = _FUEL_CELL_FAILSAFE_IE24_REDUCED_POWER;
            break;

        case _FUEL_CELL_ERR_CODE_SPM_LOST:
            _temp_failsafe = _FUEL_CELL_FAILSAFE_IE24_SPM_LOST;
            break;

        case _FUEL_CELL_ERR_CODE_PRESSURE_LOW:
            _temp_failsafe = _FUEL_CELL_FAILSAFE_IE24_PRESSURE_LOW;
            break;

        case _FUEL_CELL_ERR_CODE_BATTERY_LOW:
            _temp_failsafe = _FUEL_CELL_FAILSAFE_IE24_BATTERY_LOW;
            break;

        case _FUEL_CELL_ERR_CODE_PRESSURE_ALERT:
            _temp_failsafe = _FUEL_CELL_FAILSAFE_IE24_PRESSURE_ALERT;
            break;

        case _FUEL_CELL_ERR_CODE_START_DENIED:
            _temp_failsafe = _FUEL_CELL_FAILSAFE_IE24_START_DENIED;
            break;

        case _FUEL_CELL_ERR_CODE_SYSTEM_CRITICAL:
            _temp_failsafe = _FUEL_CELL_FAILSAFE_IE24_SYSTEM_CRITICAL;
            break;

        case _FUEL_CELL_ERR_CODE_PRESSURE_CRITICAL:
            _temp_failsafe = _FUEL_CELL_FAILSAFE_IE24_PRESSURE_CRITICAL;
            break;

        case _FUEL_CELL_ERR_CODE_BATTERY_CRITICAL:
            _temp_failsafe = _FUEL_CELL_FAILSAFE_IE24_BATTERY_CRITICAL;
            break;

        default:
            _temp_failsafe = 0;
            convert_success = false;
            break;
    }
    return convert_success;
}

AP_FuelCell *AP_FuelCell::_singleton = nullptr;

namespace AP {
    AP_FuelCell *fuelcell()
    {
        return AP_FuelCell::get_singleton();
    }
};
