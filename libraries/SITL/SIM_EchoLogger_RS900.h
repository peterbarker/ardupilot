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
  Simulator for the EchoLogger RS900

./Tools/autotest/sim_vehicle.py --gdb --debug -v APMrover2 -A --uartF=sim:echologger_rs900 --speedup=1

param set PRX_TYPE 8
param set SERIAL5_PROTOCOL 26
param set SERIAL5_BAUD 921600
reboot

*/

#pragma once

#include "SIM_Aircraft.h"

#include <SITL/SITL.h>

#include <AP_HAL/utility/RingBuffer.h>
#include "SIM_SerialDevice.h"

#include <stdio.h>

namespace SITL {

class EchoLogger_RS900 : public SerialDevice {
public:

    EchoLogger_RS900();

    // update state
    void update(const Location &loc, const Vector3f &position, const Quaternion &attitude);

private:

    SITL *_sitl;

    enum class State {
        RESET_start = 45,
        SYSINIT_start = 46,
        SYSINIT_emitting_banner_start = 47,
        SYSINIT_emitting_banner = 48,
        // FIXME: move the autobaud stuff into a separate state machine
        AUTOBAUD_start = 49,
        AUTOBAUD_wait_atsign_start = 50,
        AUTOBAUD_wait_atsign = 51,
        AUTOBAUD_start_send_sync = 52,
        AUTOBAUD_send_sync = 53,
        AUTOBAUD_expect_speed_start = 54,
        AUTOBAUD_expect_speed = 55,
        AUTOBAUD_validate_speed = 56,
        AUTOBAUD_bad_speed_start = 57,
        AUTOBAUD_send_bad_speed_er = 58,
        AUTOBAUD_send_ok_slow_start = 59,
        AUTOBAUD_send_ok_slow = 60,
        AUTOBAUD_change_baud = 61,
        AUTOBAUD_wait_100ms_after_changing_baud_start = 62,
        AUTOBAUD_wait_100ms_after_changing_baud = 63,
        AUTOBAUD_send_ok_fast_start = 64,
        AUTOBAUD_send_ok_fast = 65,
        INIT_start = 66,
        INIT_delay = 67,
        INIT_send_command_string_start = 68,
        INIT_send_command_string = 69,
        COMMAND = 70,
        WORK_start = 71,
        WORK = 72,
    };
    State state = State::RESET_start;
    uint32_t state_start_time;
    void set_state(const State newstate) {
        // ::fprintf(stderr, "SIM_RS900: moving to state (%u) from (%u)\n", (uint8_t)newstate, (uint8_t)state);
        state = newstate;
        state_start_time = AP_HAL::millis();
    }
    uint32_t time_in_state() const {
        return AP_HAL::millis() - state_start_time;
    }

    bool read_byte_from_autopilot(char &c);

    const char *send_remainder;
    const char *expect_remainder;

    const char *sync_string = "#SYNC\n";
    const char *er_string = "#ER\n";
    const char *ok_string = "#OK\n";

    const char *atsign = "@";

    enum class ExpectSpeedState {
        want_opening_angle = 37,
        want_number = 38,
        want_closing_lf = 39,
    };
    ExpectSpeedState expect_speed_state;
    void set_expect_speed_state(ExpectSpeedState newstate) {
        ::fprintf(stderr, "SIM_RS900: Moving to expected-speed-state (%u) from (%u)\n", (uint8_t)newstate, (uint8_t)expect_speed_state);
        expect_speed_state = newstate;
    }
    // bool expect_string(const char *string, const char *&string_remaining);
    bool expect_atsign();

    bool send_string(const char *string,
                     const char *&string_remaining,
                     const uint32_t chunk_size);

    bool expect_speed();

    // TODO: verify that the documentation tells us to expect \r\n
    // rather than \r here!  The device itself seems to be doing \r\n
    const char *banner_string = \
"Lots of crap\n" \
"\n" \
"Even more crap\n" \
"\n" \
"Positioning to zero... Done.\r\n" \
;

    uint32_t last_banner_chunk_sent;

    void run_state_command();
    void run_state_work();

    uint32_t wait_100ms_start_ms;

    uint32_t last_cmnd_received_ms;
    uint32_t init_delay_start_ms;

    const char *cmnd_string = "CMND\r\n";
    enum class CommandState {
        start = 1,
        get_command_start = 2,
        get_command = 3,
        send_ok_start = 4,
        send_ok = 5,
        send_er_start = 6,
        send_er = 7,
    };
    CommandState command_state;
    void set_command_state(CommandState newstate) {
        ::fprintf(stderr, "SIM_RS900: Moving to command-state (%u) from (%u)\n", (uint8_t)newstate, (uint8_t)command_state);
        command_state = newstate;
    }
    uint32_t speed;

    enum class CMND_Command {
        COMMONSETTINGS = 0,
        SCANSETTINGS = 1,
        START_AND_KEEPALIVE = 6,
        STOP = 7,
    };
    struct RS900_Command {
        int32_t magic;
        int32_t command; // 0, 1, 6, 7
        int32_t checksum; // CRC32
        int32_t size; // size of payload data
        uint32_t data[18]; // FIXME: explain where this 18 comes from
    };
    assert_storage_size<RS900_Command, 88> assert_storage_size_RS900_command;

    char cmd_from_autopilot[sizeof(struct RS900_Command)];
    uint8_t cmd_from_autopilot_len;

    bool handle_RS900_Command(const RS900_Command &cmd, uint8_t cmd_len);

    void handle_RS900_Command_COMMONSETTINGS(const RS900_Command &cmd,
                                             uint8_t cmd_len);

    void handle_RS900_Command_SCANSETTINGS(const RS900_Command &cmd,
                                           uint8_t cmd_len);

    const char *work_string = "WORK\r\n";
    enum class WorkState {
        start = 1,
        send_work_start = 2,
        send_work = 3,

        gather_data_start = 4,
        gather_data = 5,
        prepare_data_0 = 6,
        send_header_0 = 7,
        send_data_0_start = 8,
        send_data_0 = 9,
        send_footer_0_start = 10,
        send_footer_0 = 11,

        prepare_data_1 = 12,
        send_header_1 = 13,
        send_data_1_start = 14,
        send_data_1 = 15,
        send_footer_1_start = 16,
        send_footer_1 = 17,

        post_footer_1_delay_start = 18,
        post_footer_1_delay = 19,
        receive_command_start = 20,
        receive_command = 21,
    };
    WorkState work_state;
    void clear_uart(bool print = false);
    void set_work_state(WorkState newstate) {
        // ::fprintf(stderr, "SIM_RS900: Moving to work-state (%u) from (%u)\n", (uint8_t)newstate, (uint8_t)work_state);
        work_state = newstate;
    }

    uint32_t gather_data_start;

    void prepare_data();
    uint32_t work_receive_command_start_ms;
    uint32_t post_footer_1_delay_start_ms;

    uint32_t work_last_keepalive;

    enum class SendHeaderState {
        sending_magic = 81,
        sending_data_offset = 82,
        sending_data_size = 83,
        sending_samplesnum = 84,
        sending_deviceid = 85,
        sending_angle = 86,
        sending_commandid = 87,
        sending_extra_header_bytes_start = 88,
        sending_extra_header_bytes = 89,
    };
    SendHeaderState send_header_state;
    SendHeaderState send_header_post_send_uint32_state;
    void set_send_header_state(SendHeaderState newstate) {
        // ::fprintf(stderr, "SIM_RS900: Moving to send_header_state (%u) from (%u)\n", (uint8_t)newstate, (uint8_t)send_header_state);
        send_header_state = newstate;
    }
    bool handle_send_data();

    const uint8_t data_offset = 28 + 20; // 28 for known headers, 20 random unknowns

    // FIXME (duh)
    uint16_t data[10] = { 37, 4, 3, 65535, 21, 56, 23, 98, 12, 18 };
    uint16_t data_len = 10;
    uint8_t data_companded[10];
    uint16_t data_sent_offset;

    enum class SendFooterState {
        sending_timestamp = 93,
        sending_magic = 94,
        sent_magic = 95,
    };
    SendFooterState send_footer_state;
    SendFooterState send_footer_post_send_uint32_state;
    void set_send_footer_state(SendFooterState newstate) {
        // ::fprintf(stderr, "SIM_RS900: Moving to send_footer_state (%u) from (%u)\n", (uint8_t)newstate, (uint8_t)send_footer_state);
        send_footer_state = newstate;
    }

    bool handle_send_uint32();

    bool handle_send_header();
    bool handle_send_body();
    bool handle_send_footer(uint32_t footer_magic);

    union {
        uint32_t uint32;
        char bytes[4];
    } send_uint32_to_send;
    uint8_t send_uint32_bytes_remaining;

    void set_uint32_to_send(uint32_t to_send);

    uint8_t extra_header_bytes_remaining_to_send;
};

}
