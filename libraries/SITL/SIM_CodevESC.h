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
  Simulator for the Codev ESCs

./Tools/autotest/sim_vehicle.py --gdb --debug -v ArduCopter -A --uartF=sim:codevesc --speedup=1 --console

param set SERIAL5_PROTOCOL 41
param set SERIAL5_BAUD 500000
# param set SERVO_FTW_MASK 15
param set SIM_CDVESC_ENA 1
reboot

param fetch

#param set SIM_FTOWESC_FM 1  # fail mask

./Tools/autotest/autotest.py --gdb --debug build.ArduCopter fly.ArduCopter.CodevESC

*/

#pragma once

#include <AP_Param/AP_Param.h>

#include "SITL_Input.h"

#include "SIM_SerialDevice.h"

#define SIM_CODEV_DEBUGGING 0
#if SIM_CODEV_DEBUGGING
#include <stdio.h>
#define simcdv_debug(fmt, args ...)  do {::fprintf(stderr,"SIMCDESC: %s:%d: " fmt "\n", __FUNCTION__, __LINE__, ## args); } while(0)
#else
#define simcdv_debug(fmt, args ...)
#endif

#include <stdio.h>

namespace SITL {

class CodevESC : public SerialDevice {
public:

    CodevESC();

    static const uint8_t CODEV_MSG_HEADER = 0xfe;

    // update state
    void update(const class Aircraft &aircraft);

    static const AP_Param::GroupInfo var_info[];

    bool enabled() const { return _enabled.get(); }

    void update_sitl_input_pwm(struct sitl_input &input);

private:

    enum class MessageID {
        // messages or command to ESC
        CONFIG_BASIC = 0,
        CONFIG_FULL,
        RUN,
        TUNE,
        DO_CMD,
        // messages from ESC
        REQUEST_INFO,
        CONFIG_INFO_BASIC,	// simple configuration info for request from flight controller
        CONFIG_INFO_FULL,// full configuration info for request from host such as computer
        RUN_INFO,// feedback message in RUN mode
        STUDY_INFO,	// studied parameters in STUDY mode
        COMM_INFO,	// communication method info
        DEVICE_INFO,// ESC device info
        ASSIGNED_ID,	// never touch MAX_NUM
    };

    AP_Int8  _enabled;  // enable Codev ESC sim
    AP_Int32  _powered_mask;  // mask of ESCs with power

    struct PACKED Empty {
    };

    struct PACKED ConfigInfoBasicRequest {
        uint8_t  maxChannelInUse;
        uint8_t  channelMapTable[8];
        uint8_t  monitorMsgType;
        uint8_t  controlMode;
        uint16_t minChannelValue;
        uint16_t maxChannelValue;
    };

    template <typename T>
    class PACKED PackedMessage {
    public:
        PackedMessage(T _msg) :
            msg{_msg}
        {
            len = 4 + sizeof(T);
            update_checksum();
        }
        uint8_t header = CODEV_MSG_HEADER;
        uint8_t len;
        uint8_t msg_id;
        T msg;
        uint8_t checksum;

        uint8_t calculate_checksum() const WARN_IF_UNUSED {
            return crc8_dvb_update(0, (const uint8_t*)this, len-1);
        }

        void update_checksum() {
            checksum = calculate_checksum();
        }
    };

    union MessageUnion {
        MessageUnion() { }
        uint8_t buffer[255];

        PackedMessage<Empty> empty;
        PackedMessage<ConfigInfoBasicRequest> config_info_basic;

        // void update_checksum();
    };
    MessageUnion u;
    uint8_t buflen;

    // remove count bytes from the start of u.buffer
    void consume_bytes(uint8_t count);

    class ESC {
    public:

        enum class State {
            POWERED_OFF = 17,
            IN_BOOTLOADER = 18,
            POWER_UP = 19,
            RUNNING_START = 20,
            RUNNING = 21,
        };

        State state() const { return _state; }

        void set_state(State state) {
            simcdv_debug("Moving ESC.id=%u from state=%u to state=%u", (unsigned)id, (unsigned)_state, (unsigned)state);
            _state = state;
        }

        uint32_t running_start_ms;

        uint8_t runs_received;

        uint8_t id;
        uint8_t ofs;

        uint16_t pwm;

        float temperature;

        void handle_message(const MessageUnion &u);

        void power_up_handle_message(const MessageUnion &u);
        void running_handle_message(const MessageUnion &u);

    private:
        State _state = State::POWERED_OFF;

        ConfigInfoBasicRequest config;
    };

    void handle_message();

    void update_escs();
    void update_send(const class Aircraft &aircraft);
    void update_input();

    void send_esc_telemetry(const class Aircraft &aircraft);

    template <typename T>
    void send_response(const T& r);

    ESC escs[16];
};

}
