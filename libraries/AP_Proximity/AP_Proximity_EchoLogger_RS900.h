#pragma once

#include "AP_Proximity.h"
#include "AP_Proximity_Backend.h"

#include <stdio.h>

/*
  TODO:
 - settings
 - scrounge more bytes in face of serial problems
 - consolidate states?
 - fix simualator variable names

 - work out how to change in COMMAND state indefinitely
  - state diagram doesn't give a transition for this
  - current theory: send a STOP every 30s and see what happens
 - work out what's up with the required undocumented pause
 - work out what's up with the crlf thing on the banner string

 - write a void get_uint32(uint32_t &ret, WorkState &next_state);


./Tools/autotest/sim_vehicle.py -v APMrover2 -A "--uartF=uart:/dev/serial/by-id/usb-FTDI_FT232R_USB_UART_A4012SBL-if00-port0" --gdb --debug

param set PRX_TYPE 7
param set SERIAL5_PROTOCOL 24.000000
param set SERIAL5_BAUD 480600
reboot

*/

class AP_Proximity_EchoLogger_RS900 : public AP_Proximity_Backend
{

public:
    // constructor
    AP_Proximity_EchoLogger_RS900(AP_Proximity &_frontend,
                                  AP_Proximity::Proximity_State &_state);

    // static detection function
    static bool detect();

    // update state
    void update(void) override;

    // // get maximum and minimum distances (in meters) of sensor
    float distance_max() const override { return 100; }; // FIXME
    float distance_min() const override { return 0; }; // FIXME

private:

    struct CommonSettings {
        uint32_t start_node;
        uint32_t data_format;
        uint32_t commandid;
        uint32_t central_frequency;
        uint32_t frequency_band;
        uint32_t chirp_tone;
        uint32_t pulse_length;
        uint32_t ping_interval;
        uint32_t samples;
        uint32_t sample_frequency;
        float gain;
        float tvg_slope;
        uint32_t tvg_mode;
        uint32_t tvg_time;
        uint32_t sync;
        uint32_t sync_timeout;
        float tx_power;
        float rms_tx_power;
    } commonsettings PACKED;
    assert_storage_size<CommonSettings, 72> assert_storage_size_commonsettings;

    uint32_t commonsettings_change_time = 1; // time setting were changed in ArduPilot
    uint32_t commonsettings_upload_time; // time settings changed on RS900

    struct ScanSettings {
        uint16_t sector_heading;
        uint16_t sector_width;
        uint16_t rotation_parameters;
        uint16_t stepping_mode;
        uint32_t stepping_time;
        uint32_t stepping_angle;
    } scansettings PACKED;
    assert_storage_size<ScanSettings, 16> assert_storage_size_scansettings;
    uint32_t scansettings_change_time = 1; // time setting were changed in ArduPilot
    uint32_t scansettings_upload_time; // time settings changed on RS900


    bool should_work() const;

    enum class State {
        ABORTED = 76,
        init = 77,
        waiting_for_banner = 78,
        autobaud_start = 79,
        autobaud_initial_delay = 80,
        autobaud_send_at = 81,
        autobaud_wait_sync_start = 82,
        autobaud_wait_sync = 83,
        autobaud_send_speed_start = 84,
        autobaud_send_speed = 85,
        autobaud_wait_ok_slow_start = 86,
        autobaud_wait_ok_slow = 87,
        autobaud_wait_ok_fast_start = 88,
        autobaud_wait_ok_fast = 89,
        wait_for_cmnd_start = 90, // device in init state
        wait_for_cmnd = 91,
        cmnd = 92,
        work = 93,
    };
    State state = State::init;

    uint32_t state_start_ms;

    uint32_t autobaud_initial_delay_start;

    union { // not going to want both of these at the same time:
        const char *expect_remainder;
        const char *send_remainder;
    };

    uint32_t expect_string_start_ms;

    void expect_string_start(const char *str);
    bool expect_string_timed_out();

    uint32_t last_banner_prompt_ms;

    const char *expected_banner_string = "Positioning to zero... Done.\r\n";
    const char *expected_sync_string = "#SYNC\n";
    const char *expected_ok_string = "#OK\n";
    const char *expected_cmnd_string = "CMND\r\n";
    const char *expected_work_string = "WORK\r\n";
    bool expect_string(const char *wanted_string,
                       const char *&remainder_string) const;

    bool send_string(const char *&send_remainder) const;

    char *speed_string;

    // until we tell it otherwise, the RS900 talks at 115200, and
    // returns to that after a reset.
    static const uint32_t initial_baudrate = 115200;

    void set_state(State newstate) {
        ::fprintf(stderr, "RS900: moving to state (%u) from (%u)\n", (uint8_t)newstate, (uint8_t)state);
        state = newstate;
        state_start_ms = AP_HAL::millis();
    }

    uint32_t time_in_state() const {
        return AP_HAL::millis() - state_start_ms;
    }

    AP_HAL::UARTDriver *uart = nullptr;

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
        uint32_t data[18];
    };
    assert_storage_size<RS900_Command, 88> assert_storage_size_RS900_command;

    char command_base64_encoded[120 + 2 + 1]; // math.ceil(88/3.0)*4 (+2 for crlf) + 1 for null-termination
    uint8_t command_base64_encoded_len;

    void send_command(const CMND_Command cmd, const uint32_t *data, uint8_t data_size);

    void handle_state_work();
    void handle_workstate_handle_data();
    uint32_t uint32;
    uint8_t uint32_byte_offset;

    void handle_state_cmnd();

    enum class WorkState {
        waiting_for_uint32_start = 17,
        waiting_for_uint32 = 18,
        waiting_for_header = 19,
        waiting_for_header1 = 20,
        waiting_for_header2 = 21,
        waiting_for_header3 = 22,
        waiting_for_header4 = 23,
        waiting_for_data_offset = 24,
        got_data_offset = 25,
        got_data_size = 26,
        got_samplesnum = 27,
        got_deviceid = 28,
        got_angle = 29,
        got_commandid = 30,
        waiting_for_data_start = 31,
        waiting_for_extra_header_bytes = 32,
        waiting_for_data = 33,
        waiting_for_footer_start = 34,
        waiting_for_footer_timestamp = 35,
        got_footer_timestamp = 36,
        got_footer_magic = 37,
        handle_data = 38,
        send_keepalive_start = 39,
        send_keepalive = 40,
        stop_working_start = 41,
        stop_working = 42,
        send_keepalive_send_start = 43,
        send_keepalive_send_delay = 44,
        send_keepalive_send_keepalive = 45,
    };

    void set_work_state(WorkState newstate) {
        // ::fprintf(stderr, "RS900: moving to work state (%u) from (%u)\n", (uint8_t)newstate, (uint8_t)work_state);
        work_state = newstate;
        work_state_start_ms = AP_HAL::millis();
    }

    enum class CMNDState {
        START = 16,
        IDLE = 17,
        TRANSFER_COMMONSETTINGS_START = 18,
        TRANSFER_COMMONSETTINGS = 19,
        TRANSFER_COMMONSETTINGS_WAIT_RESPONSE_START = 20,
        TRANSFER_COMMONSETTINGS_WAIT_RESPONSE = 21,
        TRANSFER_SCANSETTINGS_START = 22,
        TRANSFER_SCANSETTINGS = 23,
        TRANSFER_SCANSETTINGS_WAIT_RESPONSE_START = 24,
        TRANSFER_SCANSETTINGS_WAIT_RESPONSE = 25,
        SEND_START_WORK_START = 93,
        SEND_START_WORK = 94,
        WAIT_FOR_WORK_START = 95,
        WAIT_FOR_WORK = 96,
        SEND_CMND_KEEPALIVE_START = 97,
        SEND_CMND_KEEPALIVE = 98,
        WAIT_FOR_KEEPALIVE_OK_START = 99,
        WAIT_FOR_KEEPALIVE_OK = 100,
    };
    void set_cmnd_state(CMNDState newstate) {
        // ::fprintf(stderr, "RS900: moving to work state (%u) from (%u)\n", (uint8_t)newstate, (uint8_t)work_state);
        cmnd_state = newstate;
        cmnd_state_start_ms = AP_HAL::millis();
    }
    CMNDState cmnd_state;
    uint32_t cmnd_state_start_ms;

    uint32_t send_keepalive_delay_start;

    uint32_t lost_bytes;

    struct {
        uint32_t data_offset;
        uint32_t data_size;
        uint32_t samplesnum;
        uint32_t deviceid;
        uint32_t angle;
        uint32_t commandid;
    } header;

    struct {
        uint32_t timestamp;
        uint32_t magic;
    } footer;

    uint8_t sample_data[1040]; // FIXME, guess
    uint16_t sample_data_offset;

    uint32_t extra_header_bytes_to_read;
    uint32_t sample_data_bytes_to_read;

    WorkState work_state;
    uint32_t work_state_start_ms;
    WorkState state_after_getting_uint32;

    void send_command_start_work_keepalive();
    uint32_t last_keepalive_sent_ms;

    void send_command_commonsettings();
    void send_command_scansettings();
    void send_command_stop();
};
