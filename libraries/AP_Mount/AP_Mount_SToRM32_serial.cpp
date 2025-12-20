#include "AP_Mount_config.h"

#if HAL_MOUNT_STORM32SERIAL_ENABLED

#include "AP_Mount_SToRM32_serial.h"

#include <AP_HAL/AP_HAL.h>
#include <GCS_MAVLink/GCS_MAVLink.h>
#include <GCS_MAVLink/include/mavlink/v2.0/checksum.h>
#include <AP_Logger/AP_Logger.h>

// update mount position - should be called periodically
void AP_Mount_SToRM32_serial::update()
{
    AP_Mount_Backend::update();

    // exit immediately if not initialised
    if (!_initialised) {
        return;
    }

    read_incoming(); // read the incoming messages from the gimbal

    AP_Mount_Backend::update_mnt_target();

    // send target angles (which may be derived from other target types)
    AP_Mount_Backend::send_target_to_gimbal();

    if ((AP_HAL::millis() - _last_send) > AP_MOUNT_STORM32_SERIAL_TIMEOUT_MS) {
        _reply_type = ReplyType_UNKNOWN;
    }
}

// get attitude as a quaternion.  returns true on success
bool AP_Mount_SToRM32_serial::get_attitude_quaternion(Quaternion& att_quat)
{
    att_quat.from_euler(cd_to_rad(_current_angle.x), cd_to_rad(_current_angle.y), cd_to_rad(_current_angle.z));
    return true;
}

bool AP_Mount_SToRM32_serial::can_send() {
    uint16_t required_tx = 1;
    required_tx += sizeof(AP_Mount_SToRM32_serial::cmd_set_angles_struct);
    return (_reply_type == ReplyType_UNKNOWN) && (_uart->txspace() >= required_tx);
}


// send_target_angles
void AP_Mount_SToRM32_serial::send_target_angles(const MountAngleTarget& angle_target_rad)
{
    if (!can_send()) {
        return;
    }

    static cmd_set_angles_struct cmd_set_angles_data = {
        0xFA,
        0x0E,
        0x11,
        0, // pitch
        0, // roll
        0, // yaw
        0, // flags
        0, // type
        0, // crc
    };

    // exit immediately if not initialised
    if (!_initialised) {
        return;
    }

    if ((size_t)_uart->txspace() < sizeof(cmd_set_angles_data)) {
        return;
    }

    // send CMD_SETANGLE (Note: reversed pitch and yaw)
    cmd_set_angles_data.pitch = -degrees(angle_target_rad.pitch);
    cmd_set_angles_data.roll = degrees(angle_target_rad.roll);
    cmd_set_angles_data.yaw = -degrees(angle_target_rad.get_bf_yaw());

    uint8_t* buf = (uint8_t*)&cmd_set_angles_data;

    cmd_set_angles_data.crc = crc_calculate(&buf[1], sizeof(cmd_set_angles_data)-3);

    for (uint8_t i = 0;  i != sizeof(cmd_set_angles_data) ; i++) {
        _uart->write(buf[i]);
    }

    // store time of send
    _last_send = AP_HAL::millis();

    // we pipeline commands.  We have sent in a command to set angles,
    // now fetch data from the device:
    get_angles();
    // we expect an ACK back for the set-angles command.  A state
    // machine in read_incoming will move us to ReplyType_DATA once
    // the ACK has been received.
    _reply_type = ReplyType_ACK;
    _reply_counter = 0;
    _reply_length = get_reply_size(_reply_type);

}

void AP_Mount_SToRM32_serial::get_angles() {
    // exit immediately if not initialised
    if (!_initialised) {
        return;
    }

    if (_uart->txspace() < 1) {
        return;
    }

    _uart->write('d');
};


uint8_t AP_Mount_SToRM32_serial::get_reply_size(ReplyType reply_type) {
    switch (reply_type) {
        case ReplyType_DATA:
            return sizeof(SToRM32_reply_data_struct);
            break;
        case ReplyType_ACK:
            return sizeof(SToRM32_reply_ack_struct);
            break;
        default:
            return 0;
    }
}


void AP_Mount_SToRM32_serial::read_incoming() {
    uint8_t data;
    int16_t numc;

    numc = _uart->available();

    if (numc < 0 ) {
        return;
    }

    for (int16_t i = 0; i < numc; i++) {        // Process bytes received
        data = _uart->read();
        if (_reply_type == ReplyType_UNKNOWN) {
            continue;
        }

        _buffer[_reply_counter++] = data;
        if (_reply_counter == _reply_length) {
            parse_reply();

            switch (_reply_type) {
                case ReplyType_ACK:
                    _reply_type = ReplyType_DATA;
                    _reply_length = get_reply_size(_reply_type);
                    _reply_counter = 0;
                    break;
                case ReplyType_DATA:
                    _reply_type = ReplyType_UNKNOWN;
                    _reply_length = get_reply_size(_reply_type);
                    _reply_counter = 0;
                    break;
                default:
                    _reply_length = get_reply_size(_reply_type);
                    _reply_counter = 0;
                    break;
            }
        }
    }
}

void AP_Mount_SToRM32_serial::parse_reply() {
    uint16_t crc;
    bool crc_ok;

    switch (_reply_type) {
        case ReplyType_DATA:
            crc = crc_calculate(&_buffer[0], sizeof(_buffer.data) - 3);
            crc_ok = crc == _buffer.data.crc;
            if (!crc_ok) {
                break;
            }

            // Parse angles (Note: reversed pitch and yaw) to match ardupilot coordinate system
            _current_angle.x = _buffer.data.imu1_roll;
            _current_angle.y = -_buffer.data.imu1_pitch;
            _current_angle.z = -_buffer.data.imu1_yaw;

            // Log SToRM32 telemetry data - split into 3 messages due to field/length limits

            // @LoggerMessage: STM2
            // @Description: SToRM32 gimbal status, power, timing and IMU1 sensor data
            // @Field: TimeUS: Time since system startup
            // @Field: St: System state flags
            // @Field: Sta: Status flags (indicates enabled state, errors, etc)
            // @Field: St2: Status2 flags (additional status information)
            // @Field: I2E: I2C error count from IMU1 and IMU2
            // @Field: Vol: Battery voltage from ADC
            // @Field: Cyc: Control loop cycle time
            // @Field: GX: IMU1 gyroscope X-axis
            // @Field: GY: IMU1 gyroscope Y-axis
            // @Field: GZ: IMU1 gyroscope Z-axis
            // @Field: AX: IMU1 accelerometer X-axis (scaled by 10000)
            // @Field: AY: IMU1 accelerometer Y-axis (scaled by 10000)
            // @Field: AZ: IMU1 accelerometer Z-axis (scaled by 10000)
            AP::logger().WriteStreaming(
                "STM2",
                "TimeUS," "St," "Sta," "St2," "I2E," "Vol," "Cyc," "GX," "GY," "GZ," "AX," "AY," "AZ",
                "s"       "-"   "-"    "-"    "-"    "v"    "s"    "k"   "k"   "k"   "o"   "o"   "o",
                "F"       "-"   "-"    "-"    "-"    "C"    "F"    "B"   "B"   "B"   "C"   "C"   "C",
                "Q"       "H"   "H"    "H"    "H"    "H"    "H"    "h"   "h"   "h"   "h"   "h"   "h",
                AP_HAL::micros64(),
                _buffer.data.state,
                _buffer.data.status,
                _buffer.data.status2,
                _buffer.data.i2c_errors,
                _buffer.data.lipo_voltage,
                _buffer.data.cycle_time,
                _buffer.data.imu1_gx,
                _buffer.data.imu1_gy,
                _buffer.data.imu1_gz,
                _buffer.data.imu1_ax,
                _buffer.data.imu1_ay,
                _buffer.data.imu1_az
            );

            // @LoggerMessage: SM2A
            // @Description: SToRM32 gimbal IMU1 attitude and AHRS data
            // @Field: TimeUS: Time since system startup
            // @Field: Pit: IMU1 pitch angle in 0.01 degree units
            // @Field: Rol: IMU1 roll angle in 0.01 degree units
            // @Field: Yaw: IMU1 yaw angle in 0.01 degree units
            // @Field: AhX: AHRS rotation matrix X component (scaled by 10000)
            // @Field: AhY: AHRS rotation matrix Y component (scaled by 10000)
            // @Field: AhZ: AHRS rotation matrix Z component (scaled by 10000)
            AP::logger().WriteStreaming(
                "SM2A",
                "TimeUS," "Pit," "Rol," "Yaw," "AhX," "AhY," "AhZ",
                "s"       "d"    "d"    "d"    "d"    "d"    "d",
                "F"       "B"    "B"    "B"    "B"    "B"    "B",
                "Q"       "h"    "h"    "h"    "h"    "h"    "h",
                AP_HAL::micros64(),
                _buffer.data.imu1_pitch,
                _buffer.data.imu1_roll,
                _buffer.data.imu1_yaw,
                _buffer.data.ahrs_x,
                _buffer.data.ahrs_y,
                _buffer.data.ahrs_z
            );

            // @LoggerMessage: SM2C
            // @Description: SToRM32 gimbal control outputs, IMU2 and magnetometer data
            // @Field: TimeUS: Time since system startup
            // @Field: CPi: PID controller pitch output in 0.01 units
            // @Field: CRo: PID controller roll output in 0.01 units
            // @Field: CYa: PID controller yaw output in 0.01 units
            // @Field: IPi: Input pitch command in 0.01 degree units
            // @Field: IRo: Input roll command in 0.01 degree units
            // @Field: IYa: Input yaw command in 0.01 degree units
            // @Field: P2: IMU2 pitch angle in 0.01 degree units
            // @Field: R2: IMU2 roll angle in 0.01 degree units
            // @Field: Y2: IMU2 yaw angle in 0.01 degree units
            // @Field: MYa: Magnetometer yaw angle in 0.01 degree units
            // @Field: MPi: Magnetometer pitch angle in 0.01 degree units
            // @Field: Cnf: AHRS/IMU accelerometer confidence (scaled by 10000)
            AP::logger().WriteStreaming(
                "SM2C",
                "TimeUS," "CPi," "CRo," "CYa," "IPi," "IRo," "IYa," "P2," "R2," "Y2," "MYa," "MPi," "Cnf",
                "s"       "-"    "-"    "-"    "d"    "d"    "d"    "d"   "d"   "d"   "d"    "d"    "%",
                "F"       "0"    "0"    "0"    "B"    "B"    "B"    "B"   "B"   "B"   "B"    "B"    "B",
                "Q"       "h"    "h"    "h"    "H"    "H"    "H"    "h"   "h"   "h"   "h"    "h"    "h",
                AP_HAL::micros64(),
                _buffer.data.cpid_pitch,
                _buffer.data.cpid_roll,
                _buffer.data.cpid_yaw,
                _buffer.data.input_pitch,
                _buffer.data.input_roll,
                _buffer.data.input_yaw,
                _buffer.data.imu2_pitch,
                _buffer.data.imu2_roll,
                _buffer.data.imu2_yaw,
                _buffer.data.mag2_yaw,
                _buffer.data.mag2_pitch,
                _buffer.data.ahrs_imu_confidence
            );
            break;
        default:
            break;
    }
}
#endif // HAL_MOUNT_STORM32SERIAL_ENABLED
