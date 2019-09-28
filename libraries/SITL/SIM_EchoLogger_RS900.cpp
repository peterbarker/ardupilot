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
*/

#include "SIM_EchoLogger_RS900.h"
#include <stdio.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>

using namespace SITL;

EchoLogger_RS900::EchoLogger_RS900() :
    SerialDevice::SerialDevice()
{
}


bool EchoLogger_RS900::send_string(const char *string, const char *&string_remaining, const uint32_t chunk_size)
{
    const uint8_t to_send = MIN(strlen(string_remaining), chunk_size);
    const ssize_t written = write_to_autopilot(string_remaining, to_send);
    if (written < 0) {
        AP_HAL::panic("write failed");
    }
    if (written == 0) {
        AP_HAL::panic("eof on write");
    }
    string_remaining += written;
    return *string_remaining == 0;
}

bool EchoLogger_RS900::read_byte_from_autopilot(char &c)
{
    const ssize_t bytes_read = read_from_autopilot(&c, 1);
    if (bytes_read == -1) {
        if (errno == EWOULDBLOCK || errno == EAGAIN) {
            return false;
        }
        AP_HAL::panic("read failed");
    }
    if (bytes_read == 0) {
        return false;
    }
    if (bytes_read != 1) {
        AP_HAL::panic("bad read");
    }
    return true;
}

bool EchoLogger_RS900::expect_atsign()
{
    char c;
    if (!read_byte_from_autopilot(c)) {
        return false;
    }

    if (c == '@') {
        set_state(State::AUTOBAUD_start_send_sync);
        return true;
    }
    if (c == '\n') {
        set_state(State::RESET_start);
        return false;
    }
    // ignore anything else?
    return false;
}

bool EchoLogger_RS900::expect_speed()
{
    if (time_in_state() > 5000) {
        set_state(State::RESET_start);
        ::fprintf(stderr, "expect speed timeout\n");
        return false;
    }
    for (uint8_t i=0; i<8; i++) { // max 8 bytes at a time
        char c;
        const ssize_t bytes_read = read_from_autopilot(&c, 1);
        if (bytes_read == -1) {
            if (errno == EWOULDBLOCK || errno == EAGAIN) {
                return false;
            }
            AP_HAL::panic("read failed");
        }
        if (bytes_read == 0) {
            return false;
        }

        switch (expect_speed_state) {
        case ExpectSpeedState::want_opening_angle:
            if (c != '<') {
                continue;
            }
            set_expect_speed_state(ExpectSpeedState::want_number);
            continue;
        case ExpectSpeedState::want_number:
            if (c >= '0' && c <= '9') {
                speed *= 10;
                speed += c - '0';
                ::fprintf(stderr, "speed: %u\n", speed);
                continue;
            }
            if (c != '>') {
                ::fprintf(stderr, "bad speed char: %c\n", c);
                set_state(State::RESET_start);
                return false;
            }
            set_expect_speed_state(ExpectSpeedState::want_closing_lf);
            continue;
        case ExpectSpeedState::want_closing_lf:
            if (c != '\r') {
                ::fprintf(stderr, "bad speed char: %c\n", c);
                set_state(State::RESET_start);
                return false;
            }
            set_state(State::AUTOBAUD_validate_speed);
            return true;
        }
        AP_HAL::panic("Unhandled expect-speed state %u", (uint8_t)expect_speed_state);
    }
    return false;
}

void EchoLogger_RS900::handle_RS900_Command_COMMONSETTINGS(const RS900_Command &cmd, uint8_t cmd_len)
{
    ::fprintf(stderr, "received commonsettings! (%u bytes)\n", cmd_len);
}

void EchoLogger_RS900::handle_RS900_Command_SCANSETTINGS(const RS900_Command &cmd, uint8_t cmd_len)
{
    ::fprintf(stderr, "received scansettings! (%u bytes)\n", cmd_len);
}

bool EchoLogger_RS900::handle_RS900_Command(const RS900_Command &cmd, uint8_t cmd_len)
{
    // TODO: check crc!

    switch ((CMND_Command)cmd.command) {
    case CMND_Command::COMMONSETTINGS:
        handle_RS900_Command_COMMONSETTINGS(cmd, cmd_len);
        set_command_state(CommandState::send_ok_start);
        return true;
    case CMND_Command::SCANSETTINGS:
        handle_RS900_Command_SCANSETTINGS(cmd, cmd_len);
        set_command_state(CommandState::send_ok_start);
        return true;
    case CMND_Command::START_AND_KEEPALIVE:
        set_state(State::WORK_start);
        return true; // OK will not be sent...
    case CMND_Command::STOP:
        // TODO: is this valid when in command state?  Check device.
        set_state(State::COMMAND);
        set_command_state(CommandState::send_ok_start);
        return true;
    }
    AP_HAL::panic("Unhandled command %u", cmd.command);
}


//base64_decode - swiped from rsync
/* Decode a base64 string in-place - simple and slow algorithm.
   Return length of result. */
static size_t base64_decode(char *s)
{
        const char *b64 = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
        int bit_offset, byte_offset, idx, i, n;
        unsigned char *d = (unsigned char *)s;
        const char *p;

        n=i=0;

        while (*s && (p=strchr(b64, *s))) {
                idx = (int)(p - b64);
                byte_offset = (i*6)/8;
                bit_offset = (i*6)%8;
                d[byte_offset] &= ~((1<<(8-bit_offset))-1);
                if (bit_offset < 3) {
                        d[byte_offset] |= (idx << (2-bit_offset));
                        n = byte_offset+1;
                } else {
                        d[byte_offset] |= (idx >> (bit_offset-2));
                        d[byte_offset+1] = 0;
                        d[byte_offset+1] |= (idx << (8-(bit_offset-2))) & 0xFF;
                        n = byte_offset+2;
                }
                s++; i++;
        }

        return n;
}

void EchoLogger_RS900::run_state_command()
{
    const uint32_t now = AP_HAL::millis();
    if (now - last_cmnd_received_ms > 60000) {
        ::fprintf(stderr, "SIM_RS900: cmnd timeout; reset\n");
        set_state(State::RESET_start);
        return;
    }
    static uint32_t last_state_print_ms;
    if (now - last_state_print_ms > 10000) {
        ::fprintf(stderr, "SIM_RS900: run_state_command\n");
        last_state_print_ms = now;
    }

    switch (command_state) {
    case CommandState::start:
        set_command_state(CommandState::get_command_start);
        last_cmnd_received_ms = now;
        FALLTHROUGH;
    case CommandState::get_command_start:
        cmd_from_autopilot_len = 0;
        memset(cmd_from_autopilot, '\0', ARRAY_SIZE(cmd_from_autopilot));
        set_command_state(CommandState::get_command);
        FALLTHROUGH;
    case CommandState::send_er_start: {
        send_remainder = er_string;
        set_command_state(CommandState::send_er);
        FALLTHROUGH;
    }
    case CommandState::send_er: {
        if (!send_string(er_string, send_remainder, 10) != 0) {
            // still no null-terminator
            return;
        }
        set_command_state(CommandState::get_command);
        return; // will sort things out next call
    }
    case CommandState::send_ok_start: {
        send_remainder = ok_string;
        set_command_state(CommandState::send_ok);
        FALLTHROUGH;
    }
    case CommandState::send_ok: {
        if (!send_string(ok_string, send_remainder, 10) != 0) {
            // still no null-terminator
            return;
        }
        set_command_state(CommandState::get_command);
        FALLTHROUGH;
    }
    case CommandState::get_command: {
        while (true) {
            if (!read_byte_from_autopilot(cmd_from_autopilot[cmd_from_autopilot_len])) {
                return;
            }
            cmd_from_autopilot_len++;
            char *crlf = strstr(cmd_from_autopilot, "\r\n");
            if (crlf != nullptr) {
                // we base64-decode in place then zero the command string
                *crlf = '\0'; // null-terminate string
                ::fprintf(stderr, "Got command (%s)!\n", cmd_from_autopilot);
                const size_t decoded_len = base64_decode((char*)cmd_from_autopilot);
                const RS900_Command *cmd = (RS900_Command*)cmd_from_autopilot;
                last_cmnd_received_ms = now;
                if (handle_RS900_Command(*cmd, decoded_len)) {
                    // send OK
                    ::fprintf(stderr, "SIM_RS900: state is (%u)\n", (uint8_t)state);
                    if (state == State::WORK_start) {
                        // do not send OK if we have shifted to work.
                    } else {
                        set_command_state(CommandState::send_ok_start);
                    }
                } else {
                    // send ER
                    set_command_state(CommandState::send_er_start);
                }
                cmd_from_autopilot_len = 0;
                memset(cmd_from_autopilot, '\0', ARRAY_SIZE(cmd_from_autopilot));
                return;
            }
            // we didn't find a command in an entire buffer; shift
            // content back to make room for another character
            if (cmd_from_autopilot_len == ARRAY_SIZE(cmd_from_autopilot)) {
                memmove(cmd_from_autopilot,
                        &cmd_from_autopilot[1],
                        ARRAY_SIZE(cmd_from_autopilot)-1);
                cmd_from_autopilot_len -= 1;
            }
        }
    }
    }
}


static const uint16_t uncompand8to12[256] = { 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15,16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31,32, 33, 34, 35, 36, 37, 38, 39, 40, 41, 42, 43, 44, 45, 46, 47,48, 49, 50, 51, 52, 53, 54, 55, 56, 57, 58, 59, 60, 61, 62, 63,65, 67, 69, 71, 73, 75, 77, 79, 81, 83, 85, 87, 89, 91, 93, 95,97, 99, 101, 103, 105, 107, 109, 111, 113, 115, 117, 119, 121, 123, 125, 127,131, 135, 139, 143, 147, 151, 155, 159, 163, 167, 171, 175, 179, 183, 187, 191,195, 199, 203, 207, 211, 215, 219, 223, 227, 231, 235, 239, 243, 247, 251, 255,263, 271, 279, 287, 295, 303, 311, 319, 327, 335, 343, 351, 359, 367, 375, 383,391, 399, 407, 415, 423, 431, 439, 447, 455, 463, 471, 479, 487, 495, 503, 511,527, 543, 559, 575, 591, 607, 623, 639, 655, 671, 687, 703, 719, 735, 751, 767,783, 799, 815, 831, 847, 863, 879, 895, 911, 927, 943, 959, 975, 991, 1007, 1023,1055, 1087, 1119, 1151, 1183, 1215, 1247, 1279, 1311, 1343, 1375, 1407, 1439, 1471, 1503, 1535,1567, 1599, 1631, 1663, 1695, 1727, 1759, 1791, 1823, 1855, 1887, 1919, 1951, 1983, 2015, 2047,2111, 2175, 2239, 2303, 2367, 2431, 2495, 2559, 2623, 2687, 2751, 2815, 2879, 2943, 3007, 3071,3135, 3199, 3263, 3327, 3391, 3455, 3519, 3583, 3647, 3711, 3775, 3839, 3903, 3967, 4031, 4095};

static_assert(ARRAY_SIZE(uncompand8to12) == 256, "Correct size");

static uint8_t _compand(const uint16_t source)
{
    // binary-search
    uint8_t low = 0;
    uint8_t high = 255;
    if (source == 0) {
        return 0;
    }
    if (source >= 4095) {
        return 255;
    }
    while (low < high) {
        const uint8_t mid = (low+high+1)/2;
        const uint16_t v = uncompand8to12[mid];
        if (source >= v && source < uncompand8to12[mid+1]) {
            return mid;
        }
        if (source <=v && source > uncompand8to12[mid-1]) {
            return mid;
        }
        if (v > source) {
            high = mid-1;
        } else {
            low = mid;
        }
    }
    abort();
}

void compand12to8(uint8_t *dest, const uint16_t *source, uint8_t len)
{
    for (uint8_t i=0; i<len; i++) {
        dest[i] = _compand(source[i]);
    }
}

void EchoLogger_RS900::prepare_data()
{
    static uint32_t prepare_data_count;
    ::fprintf(stderr, "SIM_RS900: prepare_data (%u)\n", prepare_data_count++);
    compand12to8(data_companded, data, ARRAY_SIZE(data));
}

void EchoLogger_RS900::clear_uart(bool print)
{
    while (true) {
        char buffer[128];
        ssize_t ret = read_from_autopilot(buffer, sizeof(buffer));
        if (ret == -1) {
            if (errno == EWOULDBLOCK || errno == EAGAIN) {
                return;
            }
            AP_HAL::panic("SIM_RS900: read failed");
        }
        if (ret == 0) {
            break;
        }
        if (print) {
            ::fprintf(stderr, "SIM_RS900: Dropping bytes\n");
        }
    }
}

bool EchoLogger_RS900::handle_send_uint32()
{
    // ::fprintf(stderr, "remaining: %u\n", send_uint32_bytes_remaining);
    const ssize_t sent = write_to_autopilot(
        &(send_uint32_to_send.bytes[4-send_uint32_bytes_remaining]),
        send_uint32_bytes_remaining);
    if (sent == -1) {
        if (errno == EWOULDBLOCK || errno == EAGAIN) {
            return false;
        }
        AP_HAL::panic("Write error");
    }
    if (sent == 0) {
        return false;
    }
    send_uint32_bytes_remaining -= sent;
    if (send_uint32_bytes_remaining) {
        return false;
    }
    return true;
}

void EchoLogger_RS900::set_uint32_to_send(uint32_t to_send)
{
    // ::fprintf(stderr, "SIM_RS900: sending uint32: %u (%0X)\n", to_send, to_send);
    send_uint32_to_send.uint32 = to_send;
    send_uint32_bytes_remaining = 4;
}

bool EchoLogger_RS900::handle_send_header()
{
    while (true) {
        if (send_uint32_bytes_remaining) {
            if (!handle_send_uint32()) {
                return false;
            }
            set_send_header_state(send_header_post_send_uint32_state);
            send_header_post_send_uint32_state = (SendHeaderState)-17; // invalidate
        }

        switch (send_header_state) {
        case SendHeaderState::sending_magic:
            // 1096040772 == 0x41544144 == ATAD
            set_uint32_to_send(1096040772);
            send_header_post_send_uint32_state = SendHeaderState::sending_data_offset;
            continue;
        case SendHeaderState::sending_data_offset: {
            set_uint32_to_send(data_offset);
            send_header_post_send_uint32_state = SendHeaderState::sending_data_size;
            continue;
        }
        case SendHeaderState::sending_data_size: {
            set_uint32_to_send(1);
            send_header_post_send_uint32_state = SendHeaderState::sending_samplesnum;
            continue;
        }
        case SendHeaderState::sending_samplesnum: {
            set_uint32_to_send(ARRAY_SIZE(data_companded));
            send_header_post_send_uint32_state = SendHeaderState::sending_deviceid;
            continue;
        }
        case SendHeaderState::sending_deviceid: {
            set_uint32_to_send(0xFFEEDDCC);
            send_header_post_send_uint32_state = SendHeaderState::sending_angle;
            continue;
        }
        case SendHeaderState::sending_angle: {
            const uint32_t angle = 0x44556677;
            set_uint32_to_send(angle);
            send_header_post_send_uint32_state = SendHeaderState::sending_commandid;
            continue;
        }
        case SendHeaderState::sending_commandid: {
            const uint32_t commandid = 0xAABBCCDD;
            set_uint32_to_send(commandid);
            send_header_post_send_uint32_state = SendHeaderState::sending_extra_header_bytes_start;
            continue;
        }
        case SendHeaderState::sending_extra_header_bytes_start:
            extra_header_bytes_remaining_to_send = data_offset - 28;
            set_send_header_state(SendHeaderState::sending_extra_header_bytes);
            // ::fprintf(stderr, "Sending %u extra header bytes\n", extra_header_bytes_remaining_to_send);
            FALLTHROUGH;
        case SendHeaderState::sending_extra_header_bytes: {
            const char padding_bytes[] = { 0, 1, 2, 3, 4, 5, 6, 7, 8, 9 };
            // note that doing things this way means you'll get random
            // things depending on how successful your writes are....
            while (extra_header_bytes_remaining_to_send) {
                const ssize_t written = write_to_autopilot(padding_bytes, MIN(sizeof(padding_bytes), extra_header_bytes_remaining_to_send));
                if (written == -1) {
                    AP_HAL::panic("Write error");
                }
                if (written == 0) {
                    return false;
                }
                extra_header_bytes_remaining_to_send -= written;
            }
            return true;
        }
        }
        AP_HAL::panic("Should not get here");
    }
}

bool EchoLogger_RS900::handle_send_data()
{
    while (data_sent_offset < data_len) {
        const ssize_t sent = write_to_autopilot((char*)&data_companded[data_sent_offset], data_len);
        if (sent == -1) {
            if (errno == EWOULDBLOCK || errno == EAGAIN) {
                return false;
            }
            AP_HAL::panic("write failed");
        }
        if (sent == 0) {
            return false;
        }
        data_sent_offset += sent;
    }
    return true;
}

bool EchoLogger_RS900::handle_send_footer(const uint32_t footer_magic)
{
    while (true) {
        if (send_uint32_bytes_remaining) {
            if (!handle_send_uint32()) {
                return false;
            }
            set_send_footer_state(send_footer_post_send_uint32_state);
            send_footer_post_send_uint32_state = (SendFooterState)-17; // invalidate
        }
        switch (send_footer_state) {
        case SendFooterState::sending_timestamp:
            set_uint32_to_send(AP_HAL::millis());
            send_footer_post_send_uint32_state = SendFooterState::sending_magic;
            continue;
        case SendFooterState::sending_magic:
            set_uint32_to_send(footer_magic);
            send_footer_post_send_uint32_state = SendFooterState::sent_magic;
            continue;
        case SendFooterState::sent_magic:
            return true;
        }
    }
}

void EchoLogger_RS900::run_state_work()
{
    const uint32_t now = AP_HAL::millis();

    static uint32_t last_state_print_ms;
    if (now - last_state_print_ms > 10000) {
        ::fprintf(stderr, "SIM_RS900: run_state_work (state=%u)\n", (uint8_t)work_state);
        last_state_print_ms = now;
    }

    if (now - work_last_keepalive > 10000) {
        // no command in 10s
        ::fprintf(stderr, "SIM_RS900: no keepalive in 10s - reset\n");
        set_state(State::RESET_start);
        return;
    }

    switch (work_state) {
    case WorkState::start:
        FALLTHROUGH;
    case WorkState::send_work_start:
        send_remainder = work_string;
        set_work_state(WorkState::send_work);
        FALLTHROUGH;
    case WorkState::send_work:
        if (!send_string(work_string, send_remainder, 10) != 0) {
            // still no null-terminator
            return;
        }
        set_work_state(WorkState::gather_data_start);
        FALLTHROUGH;

    case WorkState::gather_data_start:
        gather_data_start = now;
        set_work_state(WorkState::gather_data);
        FALLTHROUGH;

    case WorkState::gather_data:
        if (now - gather_data_start < 500) {
            // 500ms to gather data
            return;
        }
        set_work_state(WorkState::prepare_data_0);
        FALLTHROUGH;

    case WorkState::prepare_data_0:
        prepare_data();
        set_send_header_state(SendHeaderState::sending_magic);
        set_work_state(WorkState::send_header_0);
        FALLTHROUGH;

    case WorkState::send_header_0:
        if (!handle_send_header()) {
            return;
        }
        set_work_state(WorkState::send_data_0_start);
        FALLTHROUGH;

    case WorkState::send_data_0_start:
        data_sent_offset = 0;
        set_work_state(WorkState::send_data_0);
        FALLTHROUGH;

    case WorkState::send_data_0:
        if (!handle_send_data()) {
            return;
        }
        set_work_state(WorkState::send_footer_0_start);
        FALLTHROUGH;

    case WorkState::send_footer_0_start:
        set_send_footer_state(SendFooterState::sending_timestamp);
        set_work_state(WorkState::send_footer_0);
        FALLTHROUGH;

    case WorkState::send_footer_0:
        if (!handle_send_footer(809782853)) { // literally END0
            return;
        }
        set_work_state(WorkState::prepare_data_1);
        FALLTHROUGH;

    case WorkState::prepare_data_1:
        prepare_data();
        set_send_header_state(SendHeaderState::sending_magic);
        set_work_state(WorkState::send_header_1);
        FALLTHROUGH;

    case WorkState::send_header_1:
        if (!handle_send_header()) {
            return;
        }
        set_work_state(WorkState::send_data_1_start);
        FALLTHROUGH;

    case WorkState::send_data_1_start:
        data_sent_offset = 0;
        set_work_state(WorkState::send_data_1);
        FALLTHROUGH;

    case WorkState::send_data_1:
        if (!handle_send_data()) {
            return;
        }
        set_work_state(WorkState::send_footer_1_start);
        FALLTHROUGH;

    case WorkState::send_footer_1_start:
        set_send_footer_state(SendFooterState::sending_timestamp);
        set_work_state(WorkState::send_footer_1);
        FALLTHROUGH;

    case WorkState::send_footer_1:
        if (!handle_send_footer(826560069)) { // literally END1
            return;
        }
        set_work_state(WorkState::post_footer_1_delay_start);
        FALLTHROUGH;

    case WorkState::post_footer_1_delay_start:
        post_footer_1_delay_start_ms = now;
        clear_uart(); // half-duplex protocol
        set_work_state(WorkState::receive_command_start);
        FALLTHROUGH;

    case WorkState::post_footer_1_delay: // 3ms dead period, apparently...
        if (now - post_footer_1_delay_start_ms < 3) {
            clear_uart(); // half-duplex protocol
            return;
        }
        set_work_state(WorkState::receive_command_start);
        FALLTHROUGH;

    case WorkState::receive_command_start:
        work_receive_command_start_ms = now;
        // lose any partial command received from last time; TODO:
        // ensure device works like this?!
        cmd_from_autopilot_len = 0;
        memset(cmd_from_autopilot, '\0', ARRAY_SIZE(cmd_from_autopilot));
//        clear_uart();
        set_work_state(WorkState::receive_command);
        FALLTHROUGH;
    case WorkState::receive_command: {
        while (true) {
            if (now - work_receive_command_start_ms > 50) {
                ::fprintf(stderr, "SIM_RS900: returning to sending work\n");
                set_work_state(WorkState::gather_data_start);
                return;
            }
            if (!read_byte_from_autopilot(cmd_from_autopilot[cmd_from_autopilot_len])) {
                return;
            }
            ::fprintf(stderr, "SIM_RS900: workstate::receive_command: %02X\n", cmd_from_autopilot[cmd_from_autopilot_len]);
            cmd_from_autopilot_len++;
            char *crlf = strstr(cmd_from_autopilot, "\r\n");
            if (crlf != nullptr) {
                *crlf = '\0'; // null-terminate string
                cmd_from_autopilot_len = 0; // for next time
                ::fprintf(stderr, "SIM_RS900: Got work command (%s)!\n", cmd_from_autopilot);
                base64_decode((char*)cmd_from_autopilot);
                const RS900_Command *cmd = (RS900_Command*)cmd_from_autopilot;
                switch ((CMND_Command)cmd->command) {
                case CMND_Command::START_AND_KEEPALIVE:
                    ::fprintf(stderr, "SIM_RS900: keepalive received\n");
                    // FIXME: check payload is as expected
                    work_last_keepalive = now;
                    memset(cmd_from_autopilot, '\0', ARRAY_SIZE(cmd_from_autopilot));
                    cmd_from_autopilot_len = 0;
                    continue;
                case CMND_Command::STOP:
                    // FIXME: check payload is as expected
                    ::fprintf(stderr, "SIM_RS900: stop received\n");
                    set_state(State::COMMAND);
                    set_command_state(CommandState::send_ok_start);
                    memset(cmd_from_autopilot, '\0', ARRAY_SIZE(cmd_from_autopilot));
                    cmd_from_autopilot_len = 0;
                    return;
                default:
                    AP_HAL::panic("Unknown command (%u) received", (uint8_t)cmd->command);
                }
            }
            // we didn't find a command in an entire buffer; shift
            // content back to make room for another character
            if (cmd_from_autopilot_len == ARRAY_SIZE(cmd_from_autopilot)) {
                memmove(cmd_from_autopilot, &cmd_from_autopilot[1], ARRAY_SIZE(cmd_from_autopilot)-1);
                cmd_from_autopilot_len -= 1;
            }
        }
    }
    }
    AP_HAL::panic("Unhandled work state %u", (uint8_t)work_state);
}

/*
  update RS900 sensor state
 */
void EchoLogger_RS900::update(const Location &loc, const Vector3f &position, const Quaternion &attitude)
{
    if (!init_sitl_pointer()) {
        return;
    }

    const uint32_t now = AP_HAL::millis();

    switch (state) {
    case State::AUTOBAUD_bad_speed_start:
        set_state(State::AUTOBAUD_send_bad_speed_er);
        send_remainder = er_string;
        FALLTHROUGH;
    case State::AUTOBAUD_send_bad_speed_er:
        if (!send_string(er_string, send_remainder, 10) != 0) {
            // still no null-terminator
            return;
        }
        set_state(State::RESET_start);
        FALLTHROUGH;

    case State::RESET_start:
        set_state(State::SYSINIT_start);
        FALLTHROUGH;
    case State::SYSINIT_start: {
        if (time_in_state() < 10000) {
            // ::fprintf(stderr, "RS900: state reset (%ums remaining)\n", 10000 - time_in_state());
            return;
        }
        set_state(State::SYSINIT_emitting_banner_start);
        FALLTHROUGH;
    }
    case State::SYSINIT_emitting_banner_start:
        send_remainder = banner_string;
        set_state(State::SYSINIT_emitting_banner);
        FALLTHROUGH;
    case State::SYSINIT_emitting_banner: {
        // if (now - last_banner_chunk_sent < 1000) {
        //     return;
        // }
        ::fprintf(stderr, "SIM_RS900: emitting banner chunk\n");
        last_banner_chunk_sent = now;
        send_string(banner_string, send_remainder, 10);
        if (*send_remainder != 0) {
            // still no null-terminator
            return;
        }
        set_state(State::AUTOBAUD_start);
        FALLTHROUGH;
    }
    case State::AUTOBAUD_start:
        set_state(State::AUTOBAUD_wait_atsign_start);
        FALLTHROUGH;
    case State::AUTOBAUD_wait_atsign_start: {
        set_state(State::AUTOBAUD_wait_atsign);
        FALLTHROUGH;
    }
    case State::AUTOBAUD_wait_atsign: {
        if (!expect_atsign()) {
            return;
        }
        FALLTHROUGH;
    }
    case State::AUTOBAUD_start_send_sync:
        send_remainder = sync_string;
        set_state(State::AUTOBAUD_send_sync);
        FALLTHROUGH;
    case State::AUTOBAUD_send_sync:
        if (!send_string(sync_string, send_remainder, 10) != 0) {
            // still no null-terminator
            return;
        }
        set_state(State::AUTOBAUD_expect_speed_start);
        FALLTHROUGH;
    case State::AUTOBAUD_expect_speed_start:
        set_state(State::AUTOBAUD_expect_speed);
        speed = 0;
        set_expect_speed_state(ExpectSpeedState::want_opening_angle);
        FALLTHROUGH;
    case State::AUTOBAUD_expect_speed:
        // will set_state and return true if we should fall through
        if (!expect_speed()) {
            return;
        }
        FALLTHROUGH;
    case State::AUTOBAUD_validate_speed: {
        static const uint32_t valid_speeds[] = { 115200, 230400, 460800, 921600, 1000000, 2000000 };
        bool valid = false;
        for (uint8_t i=0; i<ARRAY_SIZE(valid_speeds); i++) {
            if (speed == valid_speeds[i]) {
                valid = true;
                break;
            }
        }
        if (!valid) {
            ::fprintf(stderr, "bad speed: %u\n", speed);
            set_state(State::AUTOBAUD_bad_speed_start);
            return;
        }
        set_state(State::AUTOBAUD_send_ok_slow_start);
        FALLTHROUGH;
    }
    case State::AUTOBAUD_send_ok_slow_start: {
        send_remainder = ok_string;
        set_state(State::AUTOBAUD_send_ok_slow);
        FALLTHROUGH;
    }
    case State::AUTOBAUD_send_ok_slow: {
        if (!send_string(ok_string, send_remainder, 10) != 0) {
            // still no null-terminator
            return;
        }
        set_state(State::AUTOBAUD_change_baud);
        FALLTHROUGH;
    }
    case State::AUTOBAUD_change_baud: {
        // emit sounds of grunting and straining here
        ::fprintf(stderr, "SIM_RS900: Setting baud\n");
        set_state(State::AUTOBAUD_wait_100ms_after_changing_baud_start);
        FALLTHROUGH;
    }
    case State::AUTOBAUD_wait_100ms_after_changing_baud_start: {
        wait_100ms_start_ms = now;
        set_state(State::AUTOBAUD_wait_100ms_after_changing_baud);
        FALLTHROUGH;
    }
    case State::AUTOBAUD_wait_100ms_after_changing_baud: {
        if (now - wait_100ms_start_ms < 100) {
            return;
        }
        set_state(State::AUTOBAUD_send_ok_fast_start);
        FALLTHROUGH;
    }
    case State::AUTOBAUD_send_ok_fast_start: {
        send_remainder = ok_string;
        set_state(State::AUTOBAUD_send_ok_fast);
        FALLTHROUGH;
    }
    case State::AUTOBAUD_send_ok_fast: {
        if (!send_string(ok_string, send_remainder, 10) != 0) {
            // still no null-terminator
            return;
        }
        set_state(State::INIT_start);
        FALLTHROUGH;
    }

    case State::INIT_start: {
        set_state(State::INIT_delay);
        FALLTHROUGH;
    }
    case State::INIT_delay: {
        if (now - init_delay_start_ms < 3000) {
            return;
        }
        set_state(State::INIT_send_command_string_start);
        FALLTHROUGH;
    }
    case State::INIT_send_command_string_start: {
        send_remainder = cmnd_string;
        set_state(State::INIT_send_command_string);
        FALLTHROUGH;
    }
    case State::INIT_send_command_string: {
        if (!send_string(cmnd_string, send_remainder, 10) != 0) {
            // still no null-terminator
            return;
        }
        set_command_state(CommandState::start);
        set_state(State::COMMAND);
        FALLTHROUGH;
    }
    case State::COMMAND: {
        run_state_command();
        return;
    }
    case State::WORK_start: {
        work_last_keepalive = now;
        set_work_state(WorkState::send_work_start);
        set_state(State::WORK);
        FALLTHROUGH;
    }
    case State::WORK: {
        run_state_work();
        return;
    }
    }
}
