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
  support for WitMotion serially-connected AHRS devices (e.g. HWT901B)

  The device streams fixed-length 11-byte frames: 0x55, a type byte,
  eight data bytes and a one-byte sum.  Registers are written with
  0xFF 0xAA <reg> <low> <high> and read back by writing the register
  number to the READADDR register.

  Device axes are x right, y forward, z up; its Euler angles are
  referenced to an east-north-up world.  These are converted to
  ArduPilot's body (FRD) and earth (NED) frames.
 */

#pragma once

#include "AP_ExternalAHRS_config.h"

#if AP_EXTERNAL_AHRS_WITMOTION_ENABLED

#include "AP_ExternalAHRS_backend.h"

class AP_ExternalAHRS_WitMotion : public AP_ExternalAHRS_backend
{
public:

    AP_ExternalAHRS_WitMotion(AP_ExternalAHRS *frontend, AP_ExternalAHRS::state_t &state);

    // get serial port number, -1 for not enabled
    int8_t get_port(void) const override;

    const char* get_name() const override { return "WitMotion"; }

    // accessors for AP_AHRS
    bool healthy(void) const override;
    bool initialised(void) const override;
    bool pre_arm_check(char *failure_msg, uint8_t failure_msg_len) const override;
    void get_filter_status(nav_filter_status &status) const override;

    // read and parse data from the device, and configure it
    void update() override;

    uint8_t num_gps_sensors(void) const override { return 0; }

private:

    AP_HAL::UARTDriver *uart;
    bool uart_started;
    uint32_t baudrate;
    int8_t port_num = -1;

    static constexpr uint8_t MAGIC = 0x55;
    static constexpr uint8_t FRAME_LEN = 11;

    enum class MsgType : uint8_t {
        TIME_OUTPUT             = 0x50,
        ACCELERATION_OUTPUT     = 0x51,
        ANGULAR_VELOCITY_OUTPUT = 0x52,
        ANGLE_OUTPUT            = 0x53,
        MAGNETIC_OUTPUT         = 0x54,
        PRESSURE_HEIGHT_OUTPUT  = 0x56,
        QUATERNION_OUTPUT       = 0x59,
        REGISTER_VALUES         = 0x5F,
    };

    // registers we read or write:
    enum class Register : uint8_t {
        SAVE     = 0x00,
        CONTENT  = 0x02,  // RSW: bitmask of frames to output
        RATE     = 0x03,  // RRATE: output rate
        BAUD     = 0x04,
        READADDR = 0x27,  // write a register number here to read it back
        KEY      = 0x69,  // write UNLOCK_KEY here before writing other registers
    };
    static constexpr uint16_t UNLOCK_KEY = 0xB588;

    // bits in the CONTENT register:
    enum Content : uint16_t {
        TIME   = (1U << 0),
        ACCEL  = (1U << 1),
        ANGVEL = (1U << 2),
        ANGLE  = (1U << 3),
    };

    // receive buffer for one frame
    uint8_t buf[FRAME_LEN];
    uint8_t buf_used;

    void read_from_uart();
    void handle_frame();
    void discard_bytes(uint8_t n);
    static int16_t le16(const uint8_t *p) { return int16_t(uint16_t(p[1] << 8) | p[0]); }

    void handle_acceleration(const uint8_t *data);
    void handle_angular_velocity(const uint8_t *data);
    void handle_angle(const uint8_t *data);
    void handle_register_values(const uint8_t *data);

    // most recent acceleration in the body frame, published along
    // with the next angular velocity frame
    Vector3f last_accel;
    float last_temperature;
    uint32_t last_accel_ms;
    uint32_t last_gyro_ms;
    uint32_t last_angle_ms;

    /*
      configuration state machine: find the baud rate, check the
      registers which matter to us, write any which are wrong, then
      run.  Register changes take effect immediately (seen on an
      HWT901B with firmware version 14405).
     */
    enum class WitState : uint8_t {
        AUTOBAUDING,
        READ_REGISTER,
        WAIT_REGISTER,
        WRITE_REGISTERS,
        WRITE_DELAY,
        SAVE,
        CHANGE_BAUD,
        RUNNING,
    };
    WitState witstate;

    void check_config();
    bool autobaud();
    void send_set_register(Register reg, uint16_t value);
    void send_read_register(Register reg);

    uint16_t desired_regvalue(Register reg) const;
    uint16_t desired_rate_regvalue() const;
    uint16_t desired_baud_regvalue() const;

    // the registers checked against desired_regvalue(), in the order
    // they are written.  BAUD comes last as writing it changes the
    // rate the device talks at:
    static const Register checked_registers[3];
    uint16_t checked_register_values[3];
    uint8_t next_register;

    uint32_t state_change_ms;
    bool registers_unlocked;
    bool need_baud_change;
    uint8_t autobaud_index;

    static constexpr uint32_t DESIRED_BAUD = 230400;
};

#endif  // AP_EXTERNAL_AHRS_WITMOTION_ENABLED
