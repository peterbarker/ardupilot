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
  suppport for serial connected AHRS systems

  PDF for underlying module: http://wiki.wit-motion.com/english/lib/exe/fetch.php?media=module:wt901:docs:jy901usermanualv4.pdf

 */

#pragma once

#include "AP_ExternalAHRS_backend.h"

#ifndef HAL_EXTERNAL_AHRS_WITMOTION_ENABLED
#define HAL_EXTERNAL_AHRS_WITMOTION_ENABLED HAL_EXTERNAL_AHRS_ENABLED
#endif

#if HAL_EXTERNAL_AHRS_WITMOTION_ENABLED

// this define determines whether the device can be reconfigured by
// ArduPilot into its desired configuration:
#ifndef HAL_EXTERNAL_AHRS_WITMOTION_CONFIGURATION_ENABLED
#define HAL_EXTERNAL_AHRS_WITMOTION_CONFIGURATION_ENABLED 1
#endif

#include <GCS_MAVLink/GCS_MAVLink.h>

class AP_ExternalAHRS_WitMotion: public AP_ExternalAHRS_backend
{
public:

    AP_ExternalAHRS_WitMotion(AP_ExternalAHRS *frontend, AP_ExternalAHRS::state_t &state);

    // get serial port number, -1 for not enabled
    int8_t get_port(void) const override;

    // accessors for AP_AHRS
    bool healthy(void) const override;
    bool initialised(void) const override;
    bool pre_arm_check(char *failure_msg, uint8_t failure_msg_len) const override;
    void get_filter_status(nav_filter_status &status) const override;
    void send_status_report(mavlink_channel_t chan) const override;

    // callbacks to determine support
    bool has_baro() const override { return false; }

    // check for new data
    void update() override {
        read_from_uart();
    };

private:

    void update_thread();
    bool port_open;

    AP_HAL::UARTDriver *uart;
    uint32_t baudrate;
    int8_t port_num;
    HAL_Semaphore sem;

    void read_from_uart();

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
        ANGLE_OUTPUT                 = 0x53,
        MAGNETIC_OUTPUT              = 0x54,
        PRESSURE_HEIGHT_OUTPUT       = 0x56,
        QUATERNION                   = 0x59,
    };

    class PACKED TimeOutput {
    public:
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

    class PACKED AngleOutput {
    public:
        AngleOutput(uint8_t _RollL, uint8_t _RollH, uint8_t _PitchL, uint8_t _PitchH, uint8_t _YawL, uint8_t _YawH,  uint8_t _VL, uint8_t _VH) :
            RollL{_RollL},
            RollH{_RollH},
            PitchL{_PitchL},
            PitchH{_PitchH},
            YawL{_YawL},
            YawH{_YawH},
            VL{_VL},
            VH{_VH} {}

        uint8_t msgid { (uint8_t)MsgType::ANGLE_OUTPUT };

        uint8_t RollL;
        uint8_t RollH;
        uint8_t PitchL;
        uint8_t PitchH;
        uint8_t YawL;
        uint8_t YawH;
        uint8_t VL;
        uint8_t VH;
    };

    class PACKED MagneticOutput {
    public:
        MagneticOutput(uint8_t _HxL, uint8_t _HxH, uint8_t _HyL, uint8_t _HyH, uint8_t _HzL, uint8_t _HzH,  uint8_t _TL, uint8_t _TH) :
            HxL{_HxL},
            HxH{_HxH},
            HyL{_HyL},
            HyH{_HyH},
            HzL{_HzL},
            HzH{_HzH},
            TL{_TL},
            TH{_TH} {}

        uint8_t msgid { (uint8_t)MsgType::MAGNETIC_OUTPUT };

        uint8_t HxL;
        uint8_t HxH;
        uint8_t HyL;
        uint8_t HyH;
        uint8_t HzL;
        uint8_t HzH;
        uint8_t TL;
        uint8_t TH;
    };

    class PACKED PressureHeightOutput {
    public:
        PressureHeightOutput(uint8_t _P0, uint8_t _P1, uint8_t _P2, uint8_t _P3, uint8_t _H0, uint8_t _H1,  uint8_t _H2, uint8_t _H3) :
            P0{_P0},
            P1{_P1},
            P2{_P2},
            P3{_P3},
            H0{_H0},
            H1{_H1},
            H2{_H2},
            H3{_H3} {}

        uint8_t msgid { (uint8_t)MsgType::MAGNETIC_OUTPUT };

        uint8_t P0;
        uint8_t P1;
        uint8_t P2;
        uint8_t P3;
        uint8_t H0;
        uint8_t H1;
        uint8_t H2;
        uint8_t H3;
    };

    class PACKED XQuaternion {
    public:
        XQuaternion(uint8_t _Q0L, uint8_t _Q0H, uint8_t _Q1L, uint8_t _Q1H, uint8_t _Q2L, uint8_t _Q2H,  uint8_t _Q3L, uint8_t _Q3H) :
            Q0L{_Q0L},
            Q0H{_Q0H},
            Q1L{_Q1L},
            Q1H{_Q1H},
            Q2L{_Q2L},
            Q2H{_Q2H},
            Q3L{_Q3L},
            Q3H{_Q3H} {}

        uint8_t msgid { (uint8_t)MsgType::MAGNETIC_OUTPUT };

        uint8_t Q0L;
        uint8_t Q0H;
        uint8_t Q1L;
        uint8_t Q1H;
        uint8_t Q2L;
        uint8_t Q2H;
        uint8_t Q3L;
        uint8_t Q3H;
    };

    union MessageUnion {
        MessageUnion() { }
        PackedMessage<TimeOutput> packed_time_output;
        PackedMessage<AccelerationOutput> packed_acceleration_output;
        PackedMessage<AngularVelocityOutput> packed_angularvelocity_output;
        PackedMessage<AngleOutput> packed_angle_output;
        PackedMessage<MagneticOutput> packed_magnetic_output;
        PackedMessage<PressureHeightOutput> packed_pressureheight_output;
        PackedMessage<XQuaternion> packed_quaternion;

        uint8_t receive_buf[128];  // FIXME: tighten this?

        // returns true if the message content in the buffer matches
        // the checksum in the buffer
        bool message_checksum_valid() const;
    } u;
    uint8_t _receive_buf_used;

    uint32_t last_accel_ms;
    uint32_t last_gyro_ms;

    void move_magic_in_receive_buffer(const uint8_t search_start_pos);
    void consume_bytes(const uint8_t n);

    void handle_message_content(PackedMessage<TimeOutput> p);
    void handle_message_content(PackedMessage<AngularVelocityOutput> p);
    void handle_message_content(PackedMessage<AccelerationOutput> p);

#if HAL_EXTERNAL_AHRS_WITMOTION_CONFIGURATION_ENABLED
    void check_config();
    void check_baud();

    // rate monitoring
    bool check_rates();
    bool check_message_types();
    void update_received_content_regvalue(uint8_t msgtype);

    uint16_t rate_count_gyro;
    uint32_t rate_count_time_start_ms;
    uint32_t last_rate_fix_attempt_ms;

    uint16_t desired_rate_regvalue() const;
    uint16_t desired_baud_regvalue() const;
    uint16_t desired_content_regvalue() const;

    enum class Register {
        SAVE    = 0x00,
        CONTENT = 0x02,
        RATE    = 0x03,
        BAUD    = 0x04,
    };
    void send_config_request(Register reg, uint16_t value);
    void send_config();
    uint32_t check_config_start_ms;

    // autobauding support:
    uint32_t desired_baud() const;
    uint8_t last_autobaud_offset;
    uint32_t last_autobaud_begin_ms;

    enum class State {
        AUTOBAUDING,
        RUNNING,
        CHECKING_CONFIG,
        NEED_CONFIG,
        NEED_REPOWER
    };
    State state = State::AUTOBAUDING;

    uint32_t last_power_cycle_message_ms;

    enum Content {
        TIME          = (1U << 0),
        ACCEL         = (1U << 1),
        ANGVEL        = (1U << 2),
        ANGLE         = (1U << 3),
        MAGNETIC      = (1U << 4),
        ATMO          = (1U << 6),
        QUAT          = (1U << 9),
    };
    uint16_t msg_received;
    bool bad_message_received = false;
#endif  // HAL_EXTERNAL_AHRS_WITMOTION_CONFIGURATION_ENABLED
};

#endif // HAL_EXTERNAL_AHRS_ENABLED
