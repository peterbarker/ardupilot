/*
  GimbalAI ("GIMBAL-AI" ISR pod) gimbal driver using custom serial protocol

  Implemented from "Product Communication Protocol V1.13" (QD/QP/QG series).

  Two frame directions share the link but use different headers so they can
  never be confused:

  Remote control data frame (autopilot -> pod), fixed 74 bytes:
  -------------------------------------------------------------------------------------------
  Field        Byte(1-based)  Type     Description
  -------------------------------------------------------------------------------------------
  Header       1~2            u8 x2    0x55 0xAA
  Length       3              u8       whole frame length (=74)
  Frame count  4              u8       incremental counter
  C1 pkg id    5              u8       0x01 = system mode
  C1 instr     6~7            u8 x2    system mode value (repeated)
  C1 params    8~17           u8 x10   per-mode parameters
  C2 pkg id    18             u8       sensor/feature package id (0x11,0x31,0x42,0x61,0x71...)
  C2 instr     19~20          u8 x2    command within package (repeated)
  C2 params    21~30          u8 x10   per-command parameters
  Carrier data 31~73          ...      date/time, GPS validity, vehicle attitude & position
  Checksum     74             u8       sum of all previous bytes, lower 8 bits

  Telemetry status frame (pod -> autopilot):
  -------------------------------------------------------------------------------------------
  Header       1~2            u8 x2    0xAA 0x55
  Length       3              u8       whole frame length
  Frame id     4              u8       0x11 = S11 status, 0xE0 = QS0 device version
  Payload      5~(n-1)        ...      see process_packet()
  Checksum     n              u8       sum of all previous bytes, lower 8 bits

  Multi-byte values are little-endian.  Angles use LSB = 360/65536 deg.
 */

#pragma once

#include "AP_Mount_config.h"

#if HAL_MOUNT_GIMBALAI_ENABLED

#include "AP_Mount_Backend_Serial.h"

#include <AP_HAL/AP_HAL.h>
#include <AP_Math/AP_Math.h>
#include <AP_Common/AP_Common.h>
#include <AP_HAL/utility/sparse-endian.h>

#define AP_MOUNT_GIMBALAI_PACKETLEN_MAX 96  // max bytes in a telemetry frame from the pod (QS0 is 84)

class AP_Mount_GimbalAI : public AP_Mount_Backend_Serial
{

public:
    // Constructor
    using AP_Mount_Backend_Serial::AP_Mount_Backend_Serial;

    /* Do not allow copies */
    CLASS_NO_COPY(AP_Mount_GimbalAI);

    // update mount position - should be called periodically
    void update() override;

    // return true if healthy
    bool healthy() const override;

    // the pod reports roll attitude but does not accept roll targets
    bool has_roll_control() const override { return false; }

    // has_pan_control - returns true if this mount can control its pan (required for multicopters)
    bool has_pan_control() const override { return yaw_range_valid(); };

    //
    // camera controls
    //

    // take a picture.  returns true on success
    bool take_picture() override;

    // start or stop video recording.  returns true on success
    bool record_video(bool start_recording) override;

    // set zoom specified as a rate or percentage
    bool set_zoom(ZoomType zoom_type, float zoom_value) override;

    // set focus specified as rate, percentage or auto
    // focus in = -1, focus hold = 0, focus out = 1
    SetFocusResult set_focus(FocusType focus_type, float focus_value) override;

    // set tracking to none, point or rectangle (see TrackingType enum)
    // if POINT only p1 is used, if RECTANGLE then p1 is top-left, p2 is bottom-right
    // p1,p2 are in range 0 to 1.  0 is left or top, 1 is right or bottom
    bool set_tracking(TrackingType tracking_type, const Vector2f& p1, const Vector2f& p2) override;

    // set camera lens as a value (sensor channel) from 0 to 4
    bool set_lens(uint8_t lens) override;

    bool has_camera_information() const override { return true; }
    // return camera vendor name
    void get_camera_vendor_name(char *buf, uint8_t buflen) const override { strncpy(buf, "GimbalAI", buflen); }
    // return camera model name
    void get_camera_model_name(char *buf, uint8_t buflen) const override {
        if (!_got_device_info) {
            return;
        }
        strncpy(buf, _model_name, buflen);
    }
    // return camera firmware version
    uint32_t get_camera_firmware_version() const override { return _firmware_version; }
    // return current camera lens ID
    uint8_t get_camera_lens_id() const override { return _image_sensor; }
    // return camera capability flags
    uint32_t get_camera_cap_flags() const override {
        return (CAMERA_CAP_FLAGS_CAPTURE_VIDEO |
                CAMERA_CAP_FLAGS_CAPTURE_IMAGE |
                CAMERA_CAP_FLAGS_HAS_BASIC_ZOOM |
                CAMERA_CAP_FLAGS_HAS_BASIC_FOCUS |
                CAMERA_CAP_FLAGS_HAS_TRACKING_POINT |
                CAMERA_CAP_FLAGS_HAS_TRACKING_RECTANGLE);
    }

    // send camera settings message to GCS
    void send_camera_settings(mavlink_channel_t chan) const override;

    //
    // rangefinder
    //

    // get rangefinder distance.  Returns true on success
    bool get_rangefinder_distance(float& distance_m) const override;

    // enable/disable rangefinder.  Returns true on success
    bool set_rangefinder_enable(bool enable) override;

protected:

    // GimbalAI can be sent either angles or rates
    uint8_t natively_supported_mount_target_types() const override {
        return NATIVE_ANGLES_AND_RATES_ONLY;
    };

    // get attitude as a quaternion.  returns true on success
    bool get_attitude_quaternion(Quaternion& att_quat) override;

private:

    // C1 control package ids (byte 5 of control frame)
    enum class C1Package : uint8_t {
        SYSTEM_MODE = 0x01,
    };

    // C1#M system mode values (byte 6 of control frame)
    enum class SystemMode : uint8_t {
        DO_NOTHING       = 0x00,    // keep servo state unchanged
        MOTOR_OFF        = 0x01,
        RETURN_TO_CENTER = 0x03,    // RES: azimuth 0, pitch 0
        RATE_SERVO       = 0x04,    // RATF: azimuth & pitch angular rate
        ANGLE_FRAME      = 0x05,    // GIMF: body-frame (relative to nose) angle
        ANGLE_SPATIAL    = 0x06,    // EULF: earth-frame (spatial) angle
        CLICK_TO_TRACK   = 0x10,    // TRA: enter tracking at a target point
        BOX_TRACK        = 0x11,    // CTR: enter tracking with a selection box
    };

    // C2 control package ids (byte 18 of control frame)
    enum class C2Package : uint8_t {
        NONE      = 0x00,
        VISIBLE1  = 0x11,   // C2#V1 visible light sensor 1
        INFRARED1 = 0x21,   // C2#I1 infrared sensor 1
        LASER1    = 0x31,   // C2#L1 laser sensor 1
        QUERY     = 0x42,   // C2#Q system query
        IMAGE     = 0x51,   // C2#P image control
        ENCODING  = 0x61,   // C2#E encoding control
        AI        = 0x71,   // C2#A AI control
    };

    // C2#V1 visible light sensor commands (byte 19)
    enum class VisibleCmd : uint8_t {
        ZOOM_IN          = 0x01,    // continuous zoom in (field of view -)
        ZOOM_OUT         = 0x02,    // continuous zoom out (field of view +)
        ZOOM_STOP        = 0x03,
        FOCUS_PLUS       = 0x10,
        FOCUS_MINUS      = 0x11,
        AUTO_FOCUS       = 0x12,
        ELECTRONIC_ZOOM  = 0x40,    // integer + decimal multiple
    };

    // C2#Q system query commands (byte 19)
    enum class QueryCmd : uint8_t {
        DEVICE_VERSION = 0x01,      // pod replies with a QS0 frame
    };

    // C2#P image control commands (byte 19)
    enum class ImageCmd : uint8_t {
        CHANNEL_SWITCH = 0x01,      // select main image sensor channel
    };

    // C2#E encoding control commands (byte 19)
    enum class EncodingCmd : uint8_t {
        TAKE_PHOTO = 0x02,          // param2: 1=single, 2=start continuous, 3=stop continuous
    };

    // C2#L1 laser sensor commands (byte 19)
    enum class LaserCmd : uint8_t {
        SINGLE     = 0x01,
        CONTINUOUS = 0x02,          // param1: frequency (5 = 5Hz)
        STOP       = 0x04,
    };

    // C2#A AI control commands (byte 19)
    enum class AICmd : uint8_t {
        AUTO_TRACK = 0x30,          // param1: 0=off, 1=on
    };

    // telemetry frame ids (byte 4 of telemetry frame)
    enum class TelemFrameId : uint8_t {
        S11 = 0x11,                 // system status frame
        QS0 = 0xE0,                 // device version information frame
    };

    // S11 track status (byte 43, bits 0~1)
    enum class TrackStatus : uint8_t {
        STOPPED      = 0x00,
        LOST         = 0x01,
        TRACKING     = 0x02,
        PRE_TRACKING = 0x03,
    };

    // parsing state for incoming telemetry frames
    enum class ParseState : uint8_t {
        WAITING_FOR_HEADER1,
        WAITING_FOR_HEADER2,
        WAITING_FOR_LENGTH,
        WAITING_FOR_FRAMEID,
        WAITING_FOR_DATA,
        WAITING_FOR_CRC,
    };

    // remote control data frame (autopilot -> pod), fixed 74 bytes
    union ControlFramePacket {
        struct PACKED {
            uint8_t header1;            // 0x55
            uint8_t header2;            // 0xAA
            uint8_t length;             // whole frame length (74)
            uint8_t frame_count;        // incremental counter
            uint8_t c1_package_id;      // 0x01 system mode
            uint8_t c1_instruction;     // SystemMode value
            uint8_t c1_instruction2;    // repeat of C1 instruction
            uint8_t c1_params[10];      // per-mode parameters
            uint8_t c2_package_id;      // C2Package
            uint8_t c2_instruction;     // command within package
            uint8_t c2_instruction2;    // repeat of C2 instruction
            uint8_t c2_params[10];      // per-command parameters
            le16_t date;                // bit0~6 year(+2000), bit7~10 month, bit11~15 day
            le32_t time_ms;             // milliseconds of the day
            uint8_t gps_validity;       // bit0 lat/lng/alt, bit1 velocity, bit2 heading, bit3 pitch/roll
            le16_t speed_east;          // LSB 0.01 m/s
            le16_t speed_north;         // LSB 0.01 m/s
            le16_t speed_up;            // LSB 0.01 m/s
            le16_t carrier_pitch;       // LSB 360/65536 deg
            le16_t carrier_roll;        // LSB 360/65536 deg
            le16_t carrier_heading;     // LSB 360/65536 deg, true north zero
            le32_t carrier_longitude;   // LSB 1e-7 deg
            le32_t carrier_latitude;    // LSB 1e-7 deg
            le32_t carrier_height;      // LSB 1e-3 m
            le16_t target_altitude;     // LSB 1 m
            le32_t image_frame_id;      // LSB 1
            uint8_t reserved[6];
            uint8_t checksum;           // sum of all previous bytes, lower 8 bits
        } content;
        uint8_t bytes[sizeof(content)];
    };

    // reading incoming telemetry frames from the pod; results held in _parsed_msg
    void read_incoming_packets();

    // process a successfully decoded telemetry frame held in _msg_buff
    void process_packet();

    // calculate the checksum (sum of bytes, lower 8 bits) of the given buffer
    uint8_t calc_checksum(const uint8_t *buf, uint8_t len) const;

    // little-endian int16 from telemetry buffer at the given 1-based spec byte (low byte)
    int16_t msg_int16(uint8_t spec_byte_low) const {
        return (int16_t)UINT16_VALUE(_msg_buff[spec_byte_low], _msg_buff[spec_byte_low-1]);
    }

    // populate the carrier (vehicle) attitude and position section of a control frame
    void fill_carrier_data(ControlFramePacket &packet);

    // build and send a complete control frame.  c1_params and c2_params must be
    // 10 bytes (nullptr means all zeros)
    bool send_control_frame(SystemMode c1_mode, const uint8_t *c1_params,
                            C2Package c2_pkg, uint8_t c2_instruction, const uint8_t *c2_params);

    // convenience: send a frame carrying only a C2 (camera/feature) command,
    // leaving the gimbal servo state unchanged (C1 = do-nothing)
    bool send_c2_command(C2Package c2_pkg, uint8_t c2_instruction, const uint8_t *c2_params);

    // send target pitch and yaw angles to the pod (overrides backend)
    void send_target_angles(const MountAngleTarget &angle_rad) override;

    // send target pitch and yaw rates to the pod (overrides backend)
    void send_target_rates(const MountRateTarget &rate_rads) override;

    // internal variables
    uint8_t _msg_buff[AP_MOUNT_GIMBALAI_PACKETLEN_MAX];  // buffer holding latest telemetry frame (incl. headers)
    uint8_t _msg_buff_len;                          // number of bytes held in msg buff

    // parser state and unpacked fields
    struct {
        uint8_t frame_len;                          // whole frame length from byte 3
        uint8_t frame_id;                           // frame id from byte 4
        ParseState state;                           // state of incoming message processing
    } _parsed_msg;

    uint8_t _frame_count;                           // outgoing control frame counter
    uint32_t _last_update_ms;                       // time angle/rate targets were last sent
    Vector3f _current_angle_rad;                    // latest angles from pod (x=roll, y=pitch, z=yaw)
    uint32_t _last_current_angle_rad_ms;            // time _current_angle_rad was updated (health)
    TrackStatus _last_track_status;                 // last tracking status from pod
    bool _recording;                                // recording status from pod
    uint8_t _image_sensor;                          // active image sensor channel from pod
    float _zoom_times;                              // zoom magnification from pod
    float _rangefinder_dist_m;                      // latest rangefinder distance (m)
    uint32_t _firmware_version;                     // firmware version parsed from QS0
    char _model_name[21] {};                        // model name from QS0, null-terminated
    bool _got_device_info;                          // true once a QS0 frame has been parsed
};

#endif // HAL_MOUNT_GIMBALAI_ENABLED
