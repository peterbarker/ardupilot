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
  Simulator for the WitMotion HWT901B serially-attached AHRS

  https://github.com/WITMOTION/HWT901B-TTL

  ./Tools/autotest/sim_vehicle.py -v ArduCopter --serial5=sim:WitMotion_HWT901B --console --map
  param set SERIAL5_PROTOCOL 36
  param set EAHRS_TYPE 12
  param set EAHRS_RATE 200
  reboot

  The simulated device starts in the configuration a real one comes
  in from the factory (time, acceleration, angular velocity, angle and
  magnetic field frames at 10Hz, 9600 baud) and acts on writes to its
  output-content, output-rate and baud registers immediately, as a
  real one does.  While the autopilot's baud rate differs from the
  device's, the device's output is lost.
*/

#pragma once

#include "SIM_SerialDevice.h"

#include <AP_Math/AP_Math.h>

namespace SITL {

class WitMotion_HWT901B : public SerialDevice {
public:

    WitMotion_HWT901B();

    // update state
    void update();

    uint32_t device_baud() const override;

private:

    void handle_commands();
    void send_frames();
    void send_frame(uint8_t type, const uint8_t data[8]);
    void send_vector_frame(uint8_t type, const Vector3f &v, float scale, int16_t extra);

    uint16_t output_rate_hz() const;

    enum class Register : uint8_t {
        SAVE     = 0x00,
        CONTENT  = 0x02,
        RATE     = 0x03,
        BAUD     = 0x04,
        READADDR = 0x27,
        KEY      = 0x69,
    };

    enum Content : uint16_t {
        TIME   = (1U << 0),
        ACCEL  = (1U << 1),
        ANGVEL = (1U << 2),
        ANGLE  = (1U << 3),
        MAG    = (1U << 4),
    };

    uint16_t registers[128];

    // bytes received from the autopilot, assembled into 5-byte commands
    uint8_t command[5];
    uint8_t command_used;

    uint32_t last_sent_us;
};

}
