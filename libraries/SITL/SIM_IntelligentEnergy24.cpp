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
/*
  Simulator for the IntelligentEnergy 2.4kWh FuelCell generator
*/

#include <AP_Math/AP_Math.h>

#include "SIM_IntelligentEnergy24.h"
#include "SITL.h"

#include <stdio.h>
#include <errno.h>

#include <GCS_MAVLink/GCS.h>

using namespace SITL;

// table of user settable parameters
const AP_Param::GroupInfo IntelligentEnergy24::var_info[] = {

    // @Param: ENABLE
    // @DisplayName: IntelligentEnergy 2.4kWh FuelCell sim enable/disable
    // @Description: Allows you to enable (1) or disable (0) the FuelCell simulator
    // @Values: 0:Disabled,1:Enabled
    // @User: Advanced
    AP_GROUPINFO("ENABLE", 0, IntelligentEnergy24, _enabled, 0),

    // @Param: CTRL_PIN
    // @DisplayName: Pin RichenPower is connectred to
    // @Description: The pin number that the RichenPower spinner servo is connected to. (start at 1)
    // @Range: 0 15
    // @User: Advanced
    // AP_GROUPINFO("CTRL", 2, RichenPower, _ctrl_pin, -1),

    AP_GROUPEND
};

IntelligentEnergy24::IntelligentEnergy24() : IntelligentEnergy::IntelligentEnergy()
{
    AP_Param::setup_object_defaults(this, var_info);
}

void IntelligentEnergy24::update(const struct sitl_input &input)
{
    if (!_enabled.get()) {
        return;
    }
    gcs().send_text(MAV_SEVERITY_INFO, "fuelcell update");
    update_send();
}

void IntelligentEnergy24::update_send()
{
    // just send a chunk of data at 1Hz:
    const uint32_t now = AP_HAL::millis();
    if (now - last_sent_ms < 1000) {
        return;
    }
    last_sent_ms = now;

    const float tank_bar = 1.23f;
    float battery_voltage = 11.22f;
    battery_voltage += (now % 1024) / 100.0f;  // I'm not sure this is physically accurate
    const int32_t pwr_out = 2344;
    const uint32_t spm_pwr = -17;
    const int32_t battery_pwr = 129;
    const uint32_t state = 0;
    const uint32_t err_code = 0;

    char message[128];
    snprintf(message, ARRAY_SIZE(message), "<%f,%f,%d,%u,%d,%u,%u>\n",
             tank_bar,
             battery_voltage,
             pwr_out,
             spm_pwr,
             battery_pwr,
             state,
             err_code);

    if ((unsigned)write_to_autopilot(message, strlen(message)) != strlen(message)) {
        AP_HAL::panic("Failed to write to autopilot: %s", strerror(errno));
    }
}
