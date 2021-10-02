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
 */

#pragma once

#include "AP_ExternalAHRS_backend.h"

#ifndef HAL_EXTERNAL_AHRS_WITMOTION_ENABLED
#define HAL_EXTERNAL_AHRS_WITMOTION_ENABLED HAL_EXTERNAL_AHRS_ENABLED
#endif

#if HAL_EXTERNAL_AHRS_WITMOTION_ENABLED

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

    uint32_t last_accel_ms;
    uint32_t last_gyro_ms;

    void move_magic_in_receive_buffer(const uint8_t search_start_pos);
    void consume_bytes(const uint8_t n);

    void handle_message_content(PackedMessage<TimeOutput> p);
    void handle_message_content(PackedMessage<AngularVelocityOutput> p);
    void handle_message_content(PackedMessage<AccelerationOutput> p);

};

#endif // HAL_EXTERNAL_AHRS_ENABLED

