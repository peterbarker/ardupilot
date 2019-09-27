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

#include "AP_Proximity_EchoLogger_RS900.h"
#include <AP_SerialManager/AP_SerialManager.h>
#include <GCS_MAVLink/GCS.h>
#include <AP_Math/crc.h>

#include <AP_HAL/AP_HAL.h>

#include <stdio.h>

/* 
   The constructor also initialises the proximity sensor. Note that this
   constructor is not called until detect() returns true, so we
   already know that we should setup the proximity sensor
*/
AP_Proximity_EchoLogger_RS900::AP_Proximity_EchoLogger_RS900(AP_Proximity &_frontend,
                                                         AP_Proximity::Proximity_State &_state) :
    AP_Proximity_Backend(_frontend, _state)
{
    const AP_SerialManager &serial_manager = AP::serialmanager();
    uart = serial_manager.find_serial(AP_SerialManager::SerialProtocol_EchoLogger_RS900, 0);
    uart->set_unbuffered_writes(true);

    // TODO: try to set these in initialisers:
    commonsettings.start_node = 1;
    commonsettings.commandid = 79;
    commonsettings.chirp_tone = 0;
    commonsettings.pulse_length = 20;
    commonsettings.ping_interval = 1000;
    commonsettings.samples = 480;
    commonsettings.sample_frequency = 100000;
    commonsettings.gain = 0.0;
    commonsettings.tvg_mode = 1;
    commonsettings.tvg_time = 80;
//    commonsettings.sync_timeout = 21;

    scansettings.sector_heading = 0; // sector scan heading 0-28800
    scansettings.sector_width = 0; // 0 for 360 0-28800
    scansettings.rotation_parameters = 0; // cw/ccw 0/1
    scansettings.stepping_mode = 0; // 0 is stop 0,1,2,4,8,16
    scansettings.stepping_time = commonsettings.ping_interval; // same as ping interval
    scansettings.stepping_angle = 0;
}

// detect if a Lightware proximity sensor is connected by looking for
// a configured serial port
bool AP_Proximity_EchoLogger_RS900::detect()
{
    return AP::serialmanager().find_serial(AP_SerialManager::SerialProtocol_EchoLogger_RS900, 0) != nullptr;
}

bool AP_Proximity_EchoLogger_RS900::send_string(const char *&remainder) const
{
    ::fprintf(stderr, "RS900: send-string: %s\n", remainder);
    if (!uart->txspace()) {
        return false;
    }
    const uint16_t to_send = MIN(strlen(remainder), uart->txspace());
    if (to_send == 0) {
        return true;
    }
    const ssize_t sent = uart->write((uint8_t*)remainder, to_send);
    if (sent == -1) {
        // do more here?
        return false;
    }
    remainder += sent;

    return *remainder == 0;
}

bool AP_Proximity_EchoLogger_RS900::expect_string(const char *wanted_string,
                                                  const char *&remainder_string) const
{
    // read the uart byte-wise, stepping through the string we want to see.
    const uint8_t bytes_to_read = MIN(128U, uart->available());
    // if (bytes_to_read != 0) {
        // ::fprintf(stderr, "bytes available: %u\n", bytes_to_read);
    // }
    for (uint8_t i=0; i<bytes_to_read; i++) {
        uint8_t c;
        if (uart->read(&c, 1) != 1) {
            return false;
        }
        ::fprintf(stderr, "RS900: read from device: (%c) (%02X) want=%c (%02X)\n", c, c, *remainder_string, *remainder_string);
        if (c != *remainder_string) {
            // back to the start...
            ::fprintf(stderr, "RS900: back to start\n");
            remainder_string = wanted_string;
            continue;
        }
        remainder_string++;
        if (*remainder_string == 0) {
            // found the null-termination in our wanted string.  IOW,
            // found banner.
            ::fprintf(stderr, "RS900: got string (%s)\n", wanted_string);
            return true;
        }
    }
    return false;
}


// base64_encode - swiped from ipxe, Copyright (C) 2009 Michael Brown <mbrown@fensystems.co.uk> - thanks!  (https://github.com/ipxe/ipxe/tree/master/src/core)

/**
 * Base64-encode data
 *
 * @v raw               Raw data
 * @v raw_len           Length of raw data
 * @v data              Buffer
 * @v len               Length of buffer
 * @ret len             Encoded length
 */
static const char base64[] =
        "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
size_t base64_encode ( const void *raw, size_t raw_len, char *data,
                       size_t len ) {
        const uint8_t *raw_bytes = ( ( const uint8_t * ) raw );
        size_t raw_bit_len = ( 8 * raw_len );
        size_t used = 0;
        unsigned int bit;
        unsigned int byte;
        unsigned int shift;
        unsigned int tmp;

        for ( bit = 0 ; bit < raw_bit_len ; bit += 6, used++ ) {
                byte = ( bit / 8 );
                shift = ( bit % 8 );
                tmp = ( raw_bytes[byte] << shift );
                if ( ( byte + 1 ) < raw_len )
                        tmp |= ( raw_bytes[ byte + 1 ] >> ( 8 - shift ) );
                tmp = ( ( tmp >> 2 ) & 0x3f );
                if ( used < len )
                        data[used] = base64[tmp];
        }
        for ( ; ( bit % 8 ) != 0 ; bit += 6, used++ ) {
                if ( used < len )
                        data[used] = '=';
        }
        if ( used < len )
                data[used] = '\0';
        if ( len )
                data[ len - 1 ] = '\0'; /* Ensure terminator exists */

        return used;
}

void AP_Proximity_EchoLogger_RS900::send_command(const CMND_Command command, const uint32_t *data, const uint8_t data_size)
{
    RS900_Command cmd{};
    cmd.magic = 1145982275;
    cmd.command = (int32_t)command;
    cmd.checksum = crc_crc32(0xFFFFFFFF, (uint8_t*)data, data_size);
    cmd.size = data_size;
    memcpy(cmd.data, data, MIN(data_size, sizeof(cmd.data)));

    // 16 here is 4*sizeof(int32_t) - magic, command, checksum and size
    base64_encode((unsigned char*)&cmd, 16 + data_size, command_base64_encoded, ARRAY_SIZE(command_base64_encoded));
    uint8_t len = strlen(command_base64_encoded);
    // CR/LF-terminate the line:
    command_base64_encoded[len++] = '\r';
    command_base64_encoded[len++] = '\n';
    command_base64_encoded[len++] = '\0';
    send_remainder = command_base64_encoded;
    // ::fprintf(stderr, "RS900: Sending command (%s)\n", send_remainder);
}

bool AP_Proximity_EchoLogger_RS900::should_work() const
{
    if (commonsettings_change_time != commonsettings_upload_time){
        // need to return to CMND mode to update common settings
        return false;
    }
    if (scansettings_change_time != scansettings_upload_time){
        // need to return to CMND mode to update scan settings
        return false;
    }
    return true;
}

void AP_Proximity_EchoLogger_RS900::handle_workstate_handle_data()
{
    static uint32_t count = 0;
    ::fprintf(stderr, "RS900: ##### handle_workstate_handle_data (%u)\n", count);
    count++;

    // this is the 50ms grace period where we can send commands to the
    // device - just start and stop ATM.
    if (!should_work()) {
        set_work_state(WorkState::stop_working_start);
        return;
    }

    const uint32_t now = AP_HAL::millis();

    if (now - last_keepalive_sent_ms > 1000) {
        //FIXME: this is the bit which is broken
        set_work_state(WorkState::send_keepalive_start);
        // set_work_state(WorkState::waiting_for_header);
    } else {
        set_work_state(WorkState::waiting_for_header);
   }
}


void AP_Proximity_EchoLogger_RS900::handle_state_work()
{
    const uint32_t now = AP_HAL::millis();

    static uint32_t last_message_sent;
    if (now - last_message_sent > 10000) {
        ::fprintf(stderr, "RS900: work mode (state=%u)\n", (unsigned)work_state);
        last_message_sent = now;
    }

    // remember to add something here such that if we haven't received
    // data in too long then we reset

    // notionally we could scrounge bytes here by rejecting before we
    // have a complete uint32

    const uint32_t header_magic = 1096040772; // == 0x41544144 == ATAD

    for (uint8_t i=0; i<128; i++) {
        switch (work_state) {
        case WorkState::stop_working_start:
            // the host sends “stop” command after the device send
            // “END1” footer to the host (Fig7). Then the device sends
            // “CMND<CR><LF>” string to the host.
            send_command_stop();
            set_work_state(WorkState::stop_working);
            FALLTHROUGH;
        case WorkState::stop_working:
            if (!send_string(send_remainder)) {
                return;
            }
            ::fprintf(stderr, "stopped work\n");
            set_state(State::wait_for_cmnd_start);
            return;
        case WorkState::waiting_for_uint32_start:
            uint32 = 0;
            uint32_byte_offset = 0;
            set_work_state(WorkState::waiting_for_uint32);
            FALLTHROUGH;
        case WorkState::waiting_for_uint32: {
            uint8_t c;
            if (uart->read(&c, 1) != 1) {
                return;
            }
            // ::fprintf(stderr, "RS900: work-read (%02X) (%c)\n", c, c);
            uint32 += c << uint32_byte_offset;
            if (uint32_byte_offset == 24) {
                set_work_state(state_after_getting_uint32);
                continue;
            }
            uint32_byte_offset += 8;
            continue;
        }
        case WorkState::waiting_for_header:
            set_work_state(WorkState::waiting_for_header1);
            FALLTHROUGH;
        case WorkState::waiting_for_header1: {
            uint8_t c;
            if (uart->read(&c, 1) != 1) {
                return;
            }
            if (c != (header_magic & 0xff)) {
                lost_bytes += 1;
                AP_HAL::panic("RS900: invalid header1 (%02X) (want=%02X)", c, header_magic & 0xff);
                set_work_state(WorkState::waiting_for_header);
                continue;
            }
            set_work_state(WorkState::waiting_for_header2);
            continue;
        }
        case WorkState::waiting_for_header2: {
            uint8_t c;
            if (uart->read(&c, 1) != 1) {
                return;
            }
            if (c != ((header_magic >> 8) & 0xff)) {
                AP_HAL::panic("RS900: invalid header2");
                lost_bytes += 2;
                set_work_state(WorkState::waiting_for_header);
                continue;
            }
            set_work_state(WorkState::waiting_for_header3);
            continue;
        }
        case WorkState::waiting_for_header3: {
            uint8_t c;
            if (uart->read(&c, 1) != 1) {
                return;
            }
            if (c != ((header_magic >> 16) & 0xff)) {
                AP_HAL::panic("RS900: invalid header3");
                lost_bytes += 3;
                set_work_state(WorkState::waiting_for_header);
                continue;
            }
            set_work_state(WorkState::waiting_for_header4);
            continue;
        }
        case WorkState::waiting_for_header4: {
            uint8_t c;
            if (uart->read(&c, 1) != 1) {
                return;
            }
            if (c != (header_magic >>24)) {
                AP_HAL::panic("RS900: invalid header4");
                lost_bytes += 4;
                set_work_state(WorkState::waiting_for_header);
                continue;
            }
            // ::fprintf(stderr, "RS900: valid header received\n");
            set_work_state(WorkState::waiting_for_data_offset);
            continue;
        }
        case WorkState::waiting_for_data_offset:
            state_after_getting_uint32 = WorkState::got_data_offset;
            set_work_state(WorkState::waiting_for_uint32_start);
            continue;
        case WorkState::got_data_offset:
            if (uint32 < 7*4) {
                // ::fprintf(stderr, "invalid data offset (%u)\n", uint32);
                lost_bytes += 8; // header + data-offset
                set_work_state(WorkState::waiting_for_header);
                continue;
            }
            if (uint32 > 512) {
                // our benchmark for "ridiculous"
                ::fprintf(stderr, "silly data offset  (%u)\n", uint32);
                lost_bytes += 12; // header + data-offset + data-size
                set_work_state(WorkState::waiting_for_header);
                continue;
            }
            header.data_offset = uint32;
            ::fprintf(stderr, "RS900: data offset: %u\n", header.data_offset);
            state_after_getting_uint32 = WorkState::got_data_size;
            set_work_state(WorkState::waiting_for_uint32_start);
            continue;
        case WorkState::got_data_size:
            if (uint32 != 1) {
                ::fprintf(stderr, "invalid sample size (%u)\n", uint32);
                lost_bytes += 12; // header + data-offset + data-size
                set_work_state(WorkState::waiting_for_header);
                continue;
            }
            header.data_size = uint32;
            ::fprintf(stderr, "RS900: data size: %u\n", header.data_size);
            state_after_getting_uint32 = WorkState::got_samplesnum;
            set_work_state(WorkState::waiting_for_uint32_start);
            continue;
        case WorkState::got_samplesnum:
            if (uint32 > ARRAY_SIZE(sample_data)) {
                ::fprintf(stderr, "samplesnum too large (allocated=%u, got=%u)\n", (unsigned)ARRAY_SIZE(sample_data), uint32);
                lost_bytes += 16;
                continue;
            }
            header.samplesnum = uint32;
            ::fprintf(stderr, "RS900: samplesnum: %u\n", header.samplesnum);
            state_after_getting_uint32 = WorkState::got_deviceid;
            set_work_state(WorkState::waiting_for_uint32_start);
            continue;
        case WorkState::got_deviceid:
            header.deviceid = uint32;
            ::fprintf(stderr, "RS900: deviceid: %u (%0X)\n", header.deviceid, header.deviceid);
            state_after_getting_uint32 = WorkState::got_angle;
            set_work_state(WorkState::waiting_for_uint32_start);
            continue;
        case WorkState::got_angle:
            // if (uint32 > 28800) {
            //     ::fprintf(stderr, "invalid angle (%u)\n", uint32);
            //     lost_bytes += 24; // header + data-offset + data-size
            //                       // + samplesnum + deviceid + angle
            //     set_work_state(WorkState::waiting_for_header);
            //     continue;
            // }
            header.angle = uint32;
            ::fprintf(stderr, "RS900: angle: %u\n", header.angle);
            state_after_getting_uint32 = WorkState::got_commandid;
            set_work_state(WorkState::waiting_for_uint32_start);
            continue;
        case WorkState::got_commandid:
            // data_offset indicates number of extra header bytes.
            // You can see here how many we actually understand...
            header.commandid = uint32;
            ::fprintf(stderr, "RS900: commandid: %u (%0X)\n", header.commandid, header.commandid);
            extra_header_bytes_to_read = header.data_offset - 28;
            ::fprintf(stderr, "RS900: %u extra header bytes to read\n", extra_header_bytes_to_read);
            if (extra_header_bytes_to_read == 0) {
                set_work_state(WorkState::waiting_for_data_start);
                continue;
            } else {
                set_work_state(WorkState::waiting_for_extra_header_bytes);
            }
            continue;
        case WorkState::waiting_for_extra_header_bytes: {
            // TODO: do a more complicated uart->read() here!
            // FIXME: read-many here
            uint8_t c;
            if (uart->read(&c, 1) != 1) {
                return;
            }
            ::fprintf(stderr, "RS900: Slurped extra header byte (%02x)\n", c);
            extra_header_bytes_to_read--;
            if (extra_header_bytes_to_read != 0) {
                continue;
            }
            set_work_state(WorkState::waiting_for_data_start);
            FALLTHROUGH;
        }
        case WorkState::waiting_for_data_start:
            sample_data_bytes_to_read = header.samplesnum * header.data_size;
            sample_data_bytes_to_read -= 36; // FIXME: WTF?!
            sample_data_offset = 0;
            set_work_state(WorkState::waiting_for_data);
            FALLTHROUGH;
        case WorkState::waiting_for_data: {
            const ssize_t read_bytes = uart->read(&sample_data[sample_data_offset], sample_data_bytes_to_read);
            if (read_bytes == 0) {
                return;
            }
            if (read_bytes == -1) {
                return; // hope this was eagain...
            }
            ::fprintf(stderr, "RS900: Slurped %u data bytes (wanted=%u)\n", (unsigned)read_bytes, sample_data_bytes_to_read);
            for (uint16_t j=0; j<read_bytes; j++) {
                ::fprintf(stderr, "%02X", sample_data[sample_data_offset+j]);
            }
            ::fprintf(stderr, "\n");
            sample_data_offset += read_bytes;
            sample_data_bytes_to_read -= read_bytes;
            if (sample_data_bytes_to_read != 0) {
                continue;
            }
            set_work_state(WorkState::waiting_for_footer_start);
            FALLTHROUGH;
        }
        case WorkState::waiting_for_footer_start: {
            set_work_state(WorkState::waiting_for_footer_timestamp);
            continue;
        }
        case WorkState::waiting_for_footer_timestamp: {
            state_after_getting_uint32 = WorkState::got_footer_timestamp;
            set_work_state(WorkState::waiting_for_uint32_start);
            continue;
        }
        case WorkState::got_footer_timestamp: {
            footer.timestamp = uint32;
            ::fprintf(stderr, "RS900: footer.timestamp %u\n", footer.timestamp);
            state_after_getting_uint32 = WorkState::got_footer_magic;
            set_work_state(WorkState::waiting_for_uint32_start);
            continue;
        }
        case WorkState::got_footer_magic:
            footer.magic = uint32;
            ::fprintf(stderr, "RS900: footer.magic 0x%X\n", footer.magic);
            // 826560069 == 0x31444E45 == 1DNE
            // 809782853 == 0x30444E45 == 0DNE
            if (footer.magic != 809782853 && footer.magic != 826560069) {
                abort();
                lost_bytes += 28 + header.data_offset + 8; // header, data, footer
                set_work_state(WorkState::waiting_for_header);
                continue;
            }
            set_work_state(WorkState::handle_data);
            FALLTHROUGH;
        case WorkState::handle_data:
            handle_workstate_handle_data();
            return;
        case WorkState::send_keepalive_start:
            set_work_state(WorkState::send_keepalive_send_start);
            FALLTHROUGH;
        case WorkState::send_keepalive_send_start:
            send_keepalive_delay_start = now;
            set_work_state(WorkState::send_keepalive_send_delay);
            FALLTHROUGH;
        case WorkState::send_keepalive_send_delay:
            // delay for 5ms for device to get ready for command
            if (now - send_keepalive_delay_start < 5) {
                return;
            }
            set_work_state(WorkState::send_keepalive_send_keepalive);
            FALLTHROUGH;
        case WorkState::send_keepalive_send_keepalive:
            // no response to this?!
            ::fprintf(stderr, "RS900: sending workstate keepalive\n");
            send_command_start_work_keepalive();
            set_work_state(WorkState::send_keepalive);
            FALLTHROUGH;
        case WorkState::send_keepalive:
            if (!send_string(send_remainder)) {
                // FIXME!  need a timeout just for sending string?
                // if (time_in_state() > 10000) {
                //     gcs().send_text(MAV_SEVERITY_INFO, "RS900: send-start-work-command timeout");
                //     set_state(State::init);
                // }
                return;
            }
            set_work_state(WorkState::waiting_for_header);
            return;
        }
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
        AP_HAL::panic("unhandled work state (%u)", (unsigned)work_state);
#endif
    }
}

void AP_Proximity_EchoLogger_RS900::send_command_start_work_keepalive()
{
    static constexpr uint32_t payload[] = {
        1
    };
    send_command(CMND_Command::START_AND_KEEPALIVE, payload, 4);
    last_keepalive_sent_ms = AP_HAL::millis();
}

void AP_Proximity_EchoLogger_RS900::send_command_stop()
{
    static constexpr uint32_t payload[] = {
        1
    };
    send_command(CMND_Command::STOP, payload, ARRAY_SIZE(payload));
    last_keepalive_sent_ms = AP_HAL::millis();
}

void AP_Proximity_EchoLogger_RS900::send_command_commonsettings()
{
    ::fprintf(stderr, "RS900: Sending commonsettings\n");
    send_command(CMND_Command::COMMONSETTINGS, (uint32_t*)&commonsettings, sizeof(commonsettings));
}

void AP_Proximity_EchoLogger_RS900::send_command_scansettings()
{
    ::fprintf(stderr, "RS900: Sending scansettings\n");
    send_command(CMND_Command::SCANSETTINGS, (uint32_t*)&scansettings, sizeof(scansettings));
}

bool AP_Proximity_EchoLogger_RS900::expect_string_timed_out()
{
    const uint32_t now = AP_HAL::millis();
    if (now - expect_string_start_ms > 5000) {
        return true;
    }
    return false;
}

void AP_Proximity_EchoLogger_RS900::expect_string_start(const char *str)
{
    expect_string_start_ms = AP_HAL::millis();
    expect_remainder = str;
}

void AP_Proximity_EchoLogger_RS900::handle_state_cmnd()
{
redo_commandstate: // used to redo without consuming any data
    const uint32_t now = AP_HAL::millis();
    switch(cmnd_state) {
    case CMNDState::START:
        cmnd_state_start_ms = now;
        set_cmnd_state(CMNDState::IDLE);
        FALLTHROUGH;
    case CMNDState::IDLE:
        // work through list of things we want to do in command mode:
        if (now - cmnd_state_start_ms > 30000) {
            // prod the device so we stay in CMND mode
            set_cmnd_state(CMNDState::SEND_CMND_KEEPALIVE_START);
            return;
        }
        if (commonsettings_change_time != commonsettings_upload_time) {
            // need to transfer common settings to device
            set_cmnd_state(CMNDState::TRANSFER_COMMONSETTINGS_START);
            goto redo_commandstate;
        }
        if (scansettings_change_time != scansettings_upload_time) {
            // need to transfer scan settings to device
            set_cmnd_state(CMNDState::TRANSFER_SCANSETTINGS_START);
            goto redo_commandstate;
        }
        // start to move the device into work mode:
        if (should_work()) {
            set_cmnd_state(CMNDState::SEND_START_WORK_START);
            return;
        }
        break;

        // states for transfering commonsettings to device:
    case CMNDState::TRANSFER_COMMONSETTINGS_START:
        send_command_commonsettings();
        set_cmnd_state(CMNDState::TRANSFER_COMMONSETTINGS);
        FALLTHROUGH;
    case CMNDState::TRANSFER_COMMONSETTINGS:
        if (!send_string(send_remainder)) {
            return;
        }
        set_cmnd_state(CMNDState::TRANSFER_COMMONSETTINGS_WAIT_RESPONSE_START);
        FALLTHROUGH;
    case CMNDState::TRANSFER_COMMONSETTINGS_WAIT_RESPONSE_START:
        expect_string_start(expected_ok_string);
        set_cmnd_state(CMNDState::TRANSFER_COMMONSETTINGS_WAIT_RESPONSE);
        FALLTHROUGH;
    case CMNDState::TRANSFER_COMMONSETTINGS_WAIT_RESPONSE:
        if (!expect_string(expected_ok_string, expect_remainder)) {
            if (expect_string_timed_out()) {
                gcs().send_text(MAV_SEVERITY_INFO, "RS900: commonsettings-get-OK timeout");
                set_state(State::init);
            }
            return;
        }
        // we got #OK
        commonsettings_upload_time = commonsettings_change_time;
        set_cmnd_state(CMNDState::IDLE);
        return;

        // states for transfering scansettings to device:
    case CMNDState::TRANSFER_SCANSETTINGS_START:
        send_command_scansettings();
        set_cmnd_state(CMNDState::TRANSFER_SCANSETTINGS);
        FALLTHROUGH;
    case CMNDState::TRANSFER_SCANSETTINGS:
        if (!send_string(send_remainder)) {
            return;
        }
        set_cmnd_state(CMNDState::TRANSFER_SCANSETTINGS_WAIT_RESPONSE_START);
        FALLTHROUGH;
    case CMNDState::TRANSFER_SCANSETTINGS_WAIT_RESPONSE_START:
        expect_string_start(expected_ok_string);
        set_cmnd_state(CMNDState::TRANSFER_SCANSETTINGS_WAIT_RESPONSE);
        FALLTHROUGH;
    case CMNDState::TRANSFER_SCANSETTINGS_WAIT_RESPONSE:
        if (!expect_string(expected_ok_string, expect_remainder)) {
            if (expect_string_timed_out()) {
                gcs().send_text(MAV_SEVERITY_INFO, "RS900: scansettings-get-OK timeout");
                set_state(State::init);
            }
            return;
        }
        // we got #OK
        scansettings_upload_time = scansettings_change_time;
        set_cmnd_state(CMNDState::IDLE);
        return;

    case CMNDState::SEND_START_WORK_START: {
        send_command_start_work_keepalive();
        set_cmnd_state(CMNDState::SEND_START_WORK);
        FALLTHROUGH;
    }
    case CMNDState::SEND_START_WORK:
        if (!send_string(send_remainder)) {
            if (time_in_state() > 10000) {
                gcs().send_text(MAV_SEVERITY_INFO, "RS900: send-start-work-command timeout");
                set_state(State::init);
            }
            return;
        }
        set_cmnd_state(CMNDState::WAIT_FOR_WORK_START);
        FALLTHROUGH;
    case CMNDState::WAIT_FOR_WORK_START:
        expect_string_start(expected_work_string);
        set_cmnd_state(CMNDState::WAIT_FOR_WORK);
        FALLTHROUGH;
    case CMNDState::WAIT_FOR_WORK:
        if (!expect_string(expected_work_string, expect_remainder)) {
            if (expect_string_timed_out()) {
                gcs().send_text(MAV_SEVERITY_INFO, "RS900: wait-work timeout");
                set_state(State::init);
            }
            return;
        }
        set_state(State::work);
        set_work_state(WorkState::waiting_for_header);
        return;
    case CMNDState::SEND_CMND_KEEPALIVE_START:
        send_command_stop();
        set_cmnd_state(CMNDState::SEND_CMND_KEEPALIVE);
        return;
    case CMNDState::SEND_CMND_KEEPALIVE:
        if (!send_string(send_remainder)) {
            // if (time_in_state() > 10000) {
            //     gcs().send_text(MAV_SEVERITY_INFO, "RS900: send-cmd-keepalive timeout");
            //     set_state(State::init);
            // }
            return;
        }
        set_cmnd_state(CMNDState::WAIT_FOR_KEEPALIVE_OK_START);
        FALLTHROUGH;
    case CMNDState::WAIT_FOR_KEEPALIVE_OK_START:
        expect_string_start(expected_ok_string);
        set_cmnd_state(CMNDState::WAIT_FOR_KEEPALIVE_OK);
        FALLTHROUGH;
    case CMNDState::WAIT_FOR_KEEPALIVE_OK:
        if (!expect_string(expected_ok_string, expect_remainder)) {
            if (expect_string_timed_out()) {
                gcs().send_text(MAV_SEVERITY_INFO, "RS900: wait-keepalive-ok timeout");
                set_state(State::init);
            }
            return;
        }
        ::fprintf(stderr, "RS900: Keepalive successful\n");
        set_cmnd_state(CMNDState::IDLE);
        return;
    }
}

// update the state of the sensor
void AP_Proximity_EchoLogger_RS900::update(void)
{
    if (uart == nullptr) {
        return;
    }

    // ::fprintf(stderr, "Echologger state: %u\n", (uint8_t)state); # 78
    const uint32_t now = AP_HAL::millis();

    switch (state) {
    case State::ABORTED:
        // special state where something has gone wrong and we're not
        // playing ball any more.
        return;
    case State::init:
        // we don't want to prompt for the banner immediately; it's
        // spat out by the device immediately after the reset stage
        last_banner_prompt_ms = now;
        uart->begin(initial_baudrate);
        expect_string_start(expected_banner_string);
        set_state(State::waiting_for_banner);
        FALLTHROUGH;
    case State::waiting_for_banner:
        if (!expect_string(expected_banner_string, expect_remainder)) {
            if (now - last_banner_prompt_ms > 5000) {
                // a CR is sufficient to prompt for the banner
                last_banner_prompt_ms = now;
                ::fprintf(stderr, "RS900: prompting for banner\n");
                uart->write("\n");
            }
            return;
        }
        // we got the banner
        set_state(State::autobaud_start);
        FALLTHROUGH;
    case State::autobaud_start:
        autobaud_initial_delay_start = now;
        set_state(State::autobaud_initial_delay);
        FALLTHROUGH;
    case State::autobaud_initial_delay:
        // undocumented requirement to pause here.... FIXME: add this
        // to the simulator
        if (now - autobaud_initial_delay_start < 1000) {
            return;
        }
        set_state(State::autobaud_send_at);
        FALLTHROUGH;
    case State::autobaud_send_at:
        // The host sends “@” character (0x64) at any speed from 115200 to 1000000 baud
        ::fprintf(stderr, "RS900: sending @\n");
        uart->write("@");
        set_state(State::autobaud_wait_sync_start);
        FALLTHROUGH;
    case State::autobaud_wait_sync_start:
        expect_string_start(expected_sync_string);
        set_state(State::autobaud_wait_sync);
        FALLTHROUGH;
    case State::autobaud_wait_sync:
        if (!expect_string(expected_sync_string, expect_remainder)) {
            if (expect_string_timed_out()) {
                gcs().send_text(MAV_SEVERITY_INFO, "RS900: sync timeout");
                set_state(State::init);
            }
            return;
        }
        // we got SYNC
        ::fprintf(stderr, "We got sync!\n");
        set_state(State::autobaud_send_speed_start);
        FALLTHROUGH;
    case State::autobaud_send_speed_start: {
        if (speed_string != nullptr) {
            // this is a shouldn't-happen
            free(speed_string);
            speed_string = nullptr;
        }
        const int16_t allocated = asprintf(
            &speed_string,
            "<%u>\r",
            AP::serialmanager().find_baudrate(AP_SerialManager::SerialProtocol_EchoLogger_RS900, 0));
        if (allocated == -1) {
            ::fprintf(stderr, "allocation failed\n");
            set_state(State::ABORTED);
            return;
        }
        expect_string_start(speed_string);
        set_state(State::autobaud_send_speed);
        FALLTHROUGH;
    }
    case State::autobaud_send_speed:
        if (!send_string(send_remainder)) {
            if (time_in_state() > 10000) {
                gcs().send_text(MAV_SEVERITY_INFO, "RS900: send-speed timeout");
                set_state(State::init);
            }
            return;
        }
        free(speed_string);
        speed_string = nullptr;
        set_state(State::autobaud_wait_ok_slow_start);
        FALLTHROUGH;
    case State::autobaud_wait_ok_slow_start:
        expect_string_start(expected_ok_string);
        set_state(State::autobaud_wait_ok_slow);
        FALLTHROUGH;
    case State::autobaud_wait_ok_slow:
        // TODO: consider also looking for #ER which is a valid
        // response to a bad baud rate here.
        if (!expect_string(expected_ok_string, expect_remainder)) {
            if (expect_string_timed_out()) {
                gcs().send_text(MAV_SEVERITY_INFO, "RS900: wait-ok-slow timeout");
                set_state(State::init);
            }
            return;
        }
        // if you are starting at this part of the state machine, it
        // might be because the following begin clears the UART buffer
        // - hopefully just before the device sends another "OK".
        // There's a 100ms pause, according to the documenation
        ::fprintf(stderr, "Changing baudrate to %u\n", AP::serialmanager().find_baudrate(AP_SerialManager::SerialProtocol_EchoLogger_RS900, 0));
        uart->begin(AP::serialmanager().find_baudrate(AP_SerialManager::SerialProtocol_EchoLogger_RS900, 0));
        set_state(State::autobaud_wait_ok_fast_start);
        FALLTHROUGH;
    case State::autobaud_wait_ok_fast_start:
        expect_string_start(expected_ok_string);
        set_state(State::autobaud_wait_ok_fast);
        FALLTHROUGH;
    case State::autobaud_wait_ok_fast:
        if (!expect_string(expected_ok_string, expect_remainder)) {
            if (expect_string_timed_out()) {
                gcs().send_text(MAV_SEVERITY_INFO, "RS900: wait-ok-fast timeout");
                set_state(State::init);
            }
            return;
        }
        set_state(State::wait_for_cmnd_start);
        FALLTHROUGH;
    case State::wait_for_cmnd_start:
        expect_string_start(expected_cmnd_string);
        set_state(State::wait_for_cmnd);
        FALLTHROUGH;
    case State::wait_for_cmnd:
        if (!expect_string(expected_cmnd_string, expect_remainder)) {
            if (expect_string_timed_out()) {
                gcs().send_text(MAV_SEVERITY_INFO, "RS900: wait-cmnd timeout");
                set_state(State::init);
            }
            return;
        }
        set_state(State::cmnd);
        set_cmnd_state(CMNDState::START);
        FALLTHROUGH;
    case State::cmnd: {
        handle_state_cmnd();
        return;
    }
    case State::work: {
        handle_state_work();
        return;
    }
    }
}
