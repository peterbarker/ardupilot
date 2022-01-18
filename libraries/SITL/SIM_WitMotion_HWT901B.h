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
  Simulator for the WitMotion HWT901B seriall-attached IMU sensor

  https://github.com/WITMOTION/HWT901B-TTL

  // SITL with simulated sensor:
  ./Tools/autotest/sim_vehicle.py -v ArduCopter --gdb --debug  -A --uartF=sim:WitMotion_HWT901B
  param set SERIAL5_PROTOCOL 43
  reboot

  // SITL with real sensor:
  DEV=/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0  # e.g.
  ./Tools/autotest/sim_vehicle.py -v ArduCopter -A --uartF=uart:$DEV
  param set SERIAL5_PROTOCOL 43
  param set SERIAL5_BAUD 9600
  reboot

*/

#pragma once

#include "SIM_Aircraft.h"

#include <SITL/SITL.h>

#include "SIM_SerialDevice.h"

namespace SITL {

class WitMotion_HWT901B : public SerialDevice {
public:

    WitMotion_HWT901B() {};

    // update state
    void update();

private:

    uint16_t desired_rate_hz = 100;  // start this off at a much lower rate

    static const uint8_t WITMOTION_MAGIC = 0x55;

    template <typename T>
    class PACKED PackedMessage {
    public:
        // constructor:
        PackedMessage(T _msg) :
            msg(_msg)
        {
            update_checksum();
        }

        uint8_t magic { WITMOTION_MAGIC };
        T msg;
        uint8_t checksum;

        uint8_t calculate_checksum() const {
            uint8_t _checksum = 0;
            for (uint8_t i=0; i<sizeof(*this)-1; i++) {
                _checksum += ((uint8_t*)this)[i];
            }
            return _checksum;
        }
        void update_checksum() {
            checksum = calculate_checksum();
        }
        bool verify_checksum() const {
            return checksum == calculate_checksum();
        }
    };

    enum class MsgType : uint8_t {
        TIME_OUTPUT                  = 0x50,
        ACCELERATION_OUTPUT          = 0x51,
        ANGULAR_VELOCITY_OUTPUT      = 0x52,
    };

    class PACKED TimeOutput {
    public:
        TimeOutput(uint8_t _YY, uint8_t _MM, uint8_t _DD, uint8_t _hh, uint8_t _mm, uint8_t _ss,  uint8_t _msL, uint8_t _msH) :
            YY{_YY}, MM{_MM}, DD{_DD}, hh{_hh}, mm{_mm}, ss{_ss}, msL{_msL}, msH{_msH} { }

        uint8_t msgid { (uint8_t)MsgType::TIME_OUTPUT };

        uint8_t YY;
        uint8_t MM;
        uint8_t DD;
        uint8_t hh;
        uint8_t mm;
        uint8_t ss;
        uint8_t msL;
        uint8_t msH;
    };

    class PACKED AccelerationOutput {
    public:
        AccelerationOutput(uint8_t _AxL, uint8_t _AxH, uint8_t _AyL, uint8_t _AyH, uint8_t _AzL, uint8_t _AzH,  uint8_t _TL, uint8_t _TH) :
            AxL{_AxL},
            AxH{_AxH},
            AyL{_AyL},
            AyH{_AyH},
            AzL{_AzL},
            AzH{_AzH},
            TL{_TL},
            TH{_TH} {}

        uint8_t msgid { (uint8_t)MsgType::ACCELERATION_OUTPUT };

        uint8_t AxL;
        uint8_t AxH;
        uint8_t AyL;
        uint8_t AyH;
        uint8_t AzL;
        uint8_t AzH;
        uint8_t TL;
        uint8_t TH;
    };

    class PACKED AngularVelocityOutput {
    public:
        AngularVelocityOutput(uint8_t _wxL, uint8_t _wxH, uint8_t _wyL, uint8_t _wyH, uint8_t _wzL, uint8_t _wzH,  uint8_t _TL, uint8_t _TH) :
            wxL{_wxL},
            wxH{_wxH},
            wyL{_wyL},
            wyH{_wyH},
            wzL{_wzL},
            wzH{_wzH},
            TL{_TL},
            TH{_TH} {}

        uint8_t msgid { (uint8_t)MsgType::ANGULAR_VELOCITY_OUTPUT };

        uint8_t wxL;
        uint8_t wxH;
        uint8_t wyL;
        uint8_t wyH;
        uint8_t wzL;
        uint8_t wzH;
        uint8_t TL;
        uint8_t TH;
    };

    union MessageUnion {
        MessageUnion() { }
        PackedMessage<TimeOutput> packed_time_output;
        PackedMessage<AccelerationOutput> packed_acceleration_output;
        PackedMessage<AngularVelocityOutput> packed_angularvelocity_output;
        uint8_t receive_buf[128];  // FIXME: tighten this?

        // returns true if the message content in the buffer matches
        // the checksum in the buffer
        bool message_checksum_valid() const;
    } u;
    uint8_t _receive_buf_used;


    uint32_t last_sent_ms;
};

}
