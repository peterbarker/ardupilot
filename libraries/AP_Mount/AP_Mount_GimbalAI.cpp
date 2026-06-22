#include "AP_Mount_config.h"

#if HAL_MOUNT_GIMBALAI_ENABLED

#include "AP_Mount_GimbalAI.h"

#include <AP_HAL/AP_HAL.h>
#include <AP_AHRS/AP_AHRS.h>
#include <AP_GPS/AP_GPS.h>
#include <AP_RTC/AP_RTC.h>
#include <GCS_MAVLink/GCS.h>

extern const AP_HAL::HAL& hal;

#define AP_MOUNT_GIMBALAI_HEADER1           0x55    // control frame header byte 1
#define AP_MOUNT_GIMBALAI_HEADER2           0xAA    // control frame header byte 2
#define AP_MOUNT_GIMBALAI_TLM_HEADER1       0xAA    // telemetry frame header byte 1
#define AP_MOUNT_GIMBALAI_TLM_HEADER2       0x55    // telemetry frame header byte 2
#define AP_MOUNT_GIMBALAI_HEALTH_TIMEOUT_MS 1000    // unhealthy if no telemetry within this time
#define AP_MOUNT_GIMBALAI_UPDATE_INTERVAL_MS 50     // resend targets at this interval (20Hz)
#define AP_MOUNT_GIMBALAI_DEG_TO_OUTPUT     (65536.0 / 360.0)   // degrees to wire angle units
#define AP_MOUNT_GIMBALAI_OUTPUT_TO_DEG     (360.0 / 65536.0)   // wire angle units to degrees
#define AP_MOUNT_GIMBALAI_ZOOM_MAX          10      // hard-coded absolute zoom times max

#define AP_MOUNT_GIMBALAI_DEBUG 0
#define debug(fmt, args ...) do { if (AP_MOUNT_GIMBALAI_DEBUG) { GCS_SEND_TEXT(MAV_SEVERITY_INFO, "GimbalAI: " fmt, ## args); } } while (0)

// write a little-endian int16 into a parameter buffer at the given offset
static void put_le16(uint8_t *buf, uint8_t ofs, int16_t value)
{
    buf[ofs] = (uint8_t)(value & 0xFF);
    buf[ofs+1] = (uint8_t)((value >> 8) & 0xFF);
}

// update mount position - should be called periodically
void AP_Mount_GimbalAI::update()
{
    AP_Mount_Backend::update();

    // exit immediately if not initialised
    if (!_initialised) {
        return;
    }

    // throttle sends of control frames
    const uint32_t now_ms = AP_HAL::millis();
    if (now_ms - _last_update_ms < AP_MOUNT_GIMBALAI_UPDATE_INTERVAL_MS) {
        return;
    }
    _last_update_ms = now_ms;

    // read incoming telemetry frames from the pod
    read_incoming_packets();

    // request device version (model + firmware) until we have it
    if (!_got_device_info) {
        send_c2_command(C2Package::QUERY, (uint8_t)QueryCmd::DEVICE_VERSION, nullptr);
    }

    // if tracking is active do not override with new servo targets
    if (_last_track_status == TrackStatus::TRACKING || _last_track_status == TrackStatus::PRE_TRACKING) {
        return;
    }

    // update based on mount mode and send angle or rate targets
    update_mnt_target();
    send_target_to_gimbal();
}

// return true if healthy
bool AP_Mount_GimbalAI::healthy() const
{
    // unhealthy until pod has been found
    if (!_initialised) {
        return false;
    }

    // unhealthy if telemetry not received recently
    const uint32_t now_ms = AP_HAL::millis();
    if (now_ms - _last_current_angle_rad_ms > AP_MOUNT_GIMBALAI_HEALTH_TIMEOUT_MS) {
        return false;
    }

    return true;
}

// get attitude as a quaternion.  returns true on success
bool AP_Mount_GimbalAI::get_attitude_quaternion(Quaternion& att_quat)
{
    att_quat.from_euler(_current_angle_rad.x, _current_angle_rad.y, _current_angle_rad.z);
    return true;
}

// reading incoming telemetry frames from the pod
void AP_Mount_GimbalAI::read_incoming_packets()
{
    // check for bytes on the serial port
    int16_t nbytes = MIN(_uart->available(), 1024U);
    if (nbytes <= 0) {
        return;
    }

    for (int16_t i = 0; i < nbytes; i++) {
        uint8_t b;
        if (!_uart->read(b)) {
            continue;
        }

        bool reset_parser = false;

        switch (_parsed_msg.state) {

        case ParseState::WAITING_FOR_HEADER1:
            if (b == AP_MOUNT_GIMBALAI_TLM_HEADER1) {
                _msg_buff[0] = b;
                _msg_buff_len = 1;
                _parsed_msg.state = ParseState::WAITING_FOR_HEADER2;
            }
            break;

        case ParseState::WAITING_FOR_HEADER2:
            if (b == AP_MOUNT_GIMBALAI_TLM_HEADER2) {
                _msg_buff[1] = b;
                _msg_buff_len = 2;
                _parsed_msg.state = ParseState::WAITING_FOR_LENGTH;
            } else {
                reset_parser = true;
            }
            break;

        case ParseState::WAITING_FOR_LENGTH:
            // whole frame length, including the 2 headers, length byte and checksum
            if (b < 5 || b > sizeof(_msg_buff)) {
                reset_parser = true;
                break;
            }
            _parsed_msg.frame_len = b;
            _msg_buff[2] = b;
            _msg_buff_len = 3;
            _parsed_msg.state = ParseState::WAITING_FOR_FRAMEID;
            break;

        case ParseState::WAITING_FOR_FRAMEID:
            _parsed_msg.frame_id = b;
            _msg_buff[3] = b;
            _msg_buff_len = 4;
            _parsed_msg.state = (_parsed_msg.frame_len > 5) ? ParseState::WAITING_FOR_DATA : ParseState::WAITING_FOR_CRC;
            break;

        case ParseState::WAITING_FOR_DATA:
            _msg_buff[_msg_buff_len++] = b;
            if (_msg_buff_len >= _parsed_msg.frame_len - 1) {
                _parsed_msg.state = ParseState::WAITING_FOR_CRC;
            }
            break;

        case ParseState::WAITING_FOR_CRC: {
            _msg_buff[_msg_buff_len++] = b;
            const uint8_t expected = calc_checksum(_msg_buff, _parsed_msg.frame_len - 1);
            if (expected == b) {
                process_packet();
            } else {
                debug("crc expected:%x got:%x", (unsigned)expected, (unsigned)b);
            }
            reset_parser = true;
            break;
        }
        }

        if (reset_parser) {
            _msg_buff_len = 0;
            _parsed_msg.state = ParseState::WAITING_FOR_HEADER1;
        }
    }
}

// process a successfully decoded telemetry frame held in _msg_buff
// note: _msg_buff[N-1] holds the 1-based spec byte N (headers included)
void AP_Mount_GimbalAI::process_packet()
{
    switch ((TelemFrameId)_parsed_msg.frame_id) {

    case TelemFrameId::S11: {
        // track status (byte 43, bits 0~1)
        const TrackStatus track_status = (TrackStatus)(_msg_buff[42] & 0x03);
        if (track_status != _last_track_status) {
            _last_track_status = track_status;
            switch (track_status) {
            case TrackStatus::STOPPED:
                GCS_SEND_TEXT(MAV_SEVERITY_INFO, "GimbalAI: tracking OFF");
                break;
            case TrackStatus::LOST:
                GCS_SEND_TEXT(MAV_SEVERITY_INFO, "GimbalAI: tracking Lost");
                break;
            case TrackStatus::TRACKING:
                GCS_SEND_TEXT(MAV_SEVERITY_INFO, "GimbalAI: tracking ON");
                break;
            case TrackStatus::PRE_TRACKING:
                GCS_SEND_TEXT(MAV_SEVERITY_INFO, "GimbalAI: tracking searching");
                break;
            }
        }

        // pod body-frame angles (LSB = 360/65536 deg).  pitch is negated on the
        // wire to match the joint-angle sign convention (negative = down)
        _last_current_angle_rad_ms = AP_HAL::millis();
        _current_angle_rad.z = radians(msg_int16(9)  * AP_MOUNT_GIMBALAI_OUTPUT_TO_DEG);   // azimuth/yaw (bytes 9-10)
        _current_angle_rad.y = -radians(msg_int16(11) * AP_MOUNT_GIMBALAI_OUTPUT_TO_DEG);  // pitch (bytes 11-12)
        _current_angle_rad.x = radians(msg_int16(13) * AP_MOUNT_GIMBALAI_OUTPUT_TO_DEG);   // roll (bytes 13-14)
        debug("r:%4.1f p:%4.1f y:%4.1f", (double)degrees(_current_angle_rad.x), (double)degrees(_current_angle_rad.y), (double)degrees(_current_angle_rad.z));

        // active image sensor channel (byte 54, bits 0~3)
        _image_sensor = _msg_buff[53] & 0x0F;

        // recording status (byte 65, bit 7)
        const bool recording = (_msg_buff[64] & 0x80) != 0;
        if (recording != _recording) {
            _recording = recording;
            GCS_SEND_TEXT(MAV_SEVERITY_INFO, "GimbalAI: recording %s", _recording ? "ON" : "OFF");
        }

        // zoom magnification (bytes 56-57, LSB = 0.1)
        _zoom_times = (uint16_t)UINT16_VALUE(_msg_buff[56], _msg_buff[55]) * 0.1;

        // laser rangefinder distance (bytes 39-40, LSB = 1m)
        _rangefinder_dist_m = (uint16_t)UINT16_VALUE(_msg_buff[39], _msg_buff[38]);
        break;
    }

    case TelemFrameId::QS0: {
        // device model occupies bytes 5-24 (20 chars), software version bytes 34-83 (50 chars)
        memset(_model_name, '\0', sizeof(_model_name));
        memcpy(_model_name, &_msg_buff[4], sizeof(_model_name)-1);
        // ensure null termination if the field used the full width
        _model_name[sizeof(_model_name)-1] = '\0';

        // parse the first three integer groups of the software version string
        // (e.g. "SV1.2.3") into major.minor.patch
        uint8_t ver[3] {};
        uint8_t ver_idx = 0;
        bool in_number = false;
        for (uint8_t i = 33; i < 83 && ver_idx < 3; i++) {
            const uint8_t c = _msg_buff[i];
            if (c >= '0' && c <= '9') {
                ver[ver_idx] = ver[ver_idx] * 10 + (c - '0');
                in_number = true;
            } else if (in_number) {
                ver_idx++;
                in_number = false;
            }
        }
        _firmware_version = ver[0] | (ver[1] << 8) | (ver[2] << 16);
        _got_device_info = true;
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "GimbalAI: %s fw:%u.%u.%u", _model_name,
                      (unsigned)ver[0], (unsigned)ver[1], (unsigned)ver[2]);
        break;
    }

    default:
        debug("Unhandled FrameId:%u", (unsigned)_parsed_msg.frame_id);
        break;
    }
}

// calculate the checksum (sum of bytes, lower 8 bits) of the given buffer
uint8_t AP_Mount_GimbalAI::calc_checksum(const uint8_t *buf, uint8_t len) const
{
    uint8_t res = 0;
    for (uint8_t i = 0; i < len; i++) {
        res += buf[i];
    }
    return res;
}

// populate the carrier (vehicle) attitude and position section of a control frame
void AP_Mount_GimbalAI::fill_carrier_data(ControlFramePacket &packet)
{
    // attitude (LSB = 360/65536 deg)
    packet.content.carrier_pitch = htole16((int16_t)(AP::ahrs().get_pitch_deg() * AP_MOUNT_GIMBALAI_DEG_TO_OUTPUT));
    packet.content.carrier_roll = htole16((int16_t)(AP::ahrs().get_roll_deg() * AP_MOUNT_GIMBALAI_DEG_TO_OUTPUT));
    packet.content.carrier_heading = htole16((int16_t)(wrap_180(AP::ahrs().get_yaw_deg()) * AP_MOUNT_GIMBALAI_DEG_TO_OUTPUT));

    // GPS validity bitmask: 0 = valid for each field.  Default everything invalid
    // then clear bits as data becomes available.
    uint8_t gps_validity = 0xFF;

    // position
    Location loc;
    int32_t alt_amsl_cm = 0;
    if (AP::ahrs().get_location(loc) && loc.get_alt_cm(Location::AltFrame::ABSOLUTE, alt_amsl_cm)) {
        packet.content.carrier_longitude = htole32(loc.lng);            // LSB 1e-7 deg
        packet.content.carrier_latitude = htole32(loc.lat);             // LSB 1e-7 deg
        packet.content.carrier_height = htole32(alt_amsl_cm * 10);      // LSB 1e-3 m
        gps_validity &= ~0x01;                                          // lat/lng/alt valid
    }

    // velocity in NED (m/s) -> ENU speeds (LSB 0.01 m/s)
    Vector3f vel_NED;
    if (AP::ahrs().get_velocity_NED(vel_NED)) {
        packet.content.speed_east = htole16((int16_t)(vel_NED.y * 100));
        packet.content.speed_north = htole16((int16_t)(vel_NED.x * 100));
        packet.content.speed_up = htole16((int16_t)(-vel_NED.z * 100));
        gps_validity &= ~0x02;                                          // velocity valid
    }
    gps_validity &= ~0x08;                                              // pitch/roll always valid
    packet.content.gps_validity = gps_validity;

#if AP_RTC_ENABLED
    uint16_t year, ms;
    uint8_t month, day, hour, min, sec;
    if (AP::rtc().get_date_and_time_utc(year, month, day, hour, min, sec, ms)) {
        const uint16_t date = ((year-2000) & 0x7F) | ((month & 0x0F) << 7) | ((day & 0x1F) << 11);
        const uint32_t time_ms = (((hour * 60 * 60) + (min * 60) + sec) * 1000UL) + ms;
        packet.content.date = htole16(date);
        packet.content.time_ms = htole32(time_ms);
    }
#endif
}

// build and send a complete control frame
bool AP_Mount_GimbalAI::send_control_frame(SystemMode c1_mode, const uint8_t *c1_params,
                                           C2Package c2_pkg, uint8_t c2_instruction, const uint8_t *c2_params)
{
    if (!_initialised) {
        return false;
    }

    ControlFramePacket packet {};
    packet.content.header1 = AP_MOUNT_GIMBALAI_HEADER1;
    packet.content.header2 = AP_MOUNT_GIMBALAI_HEADER2;
    packet.content.length = sizeof(packet.content);
    packet.content.frame_count = _frame_count++;
    packet.content.c1_package_id = (uint8_t)C1Package::SYSTEM_MODE;
    packet.content.c1_instruction = (uint8_t)c1_mode;
    packet.content.c1_instruction2 = (uint8_t)c1_mode;
    if (c1_params != nullptr) {
        memcpy(packet.content.c1_params, c1_params, sizeof(packet.content.c1_params));
    }
    packet.content.c2_package_id = (uint8_t)c2_pkg;
    packet.content.c2_instruction = c2_instruction;
    packet.content.c2_instruction2 = c2_instruction;
    if (c2_params != nullptr) {
        memcpy(packet.content.c2_params, c2_params, sizeof(packet.content.c2_params));
    }

    fill_carrier_data(packet);

    const uint8_t frame_len = sizeof(packet.content);
    packet.content.checksum = calc_checksum(packet.bytes, frame_len - 1);

    // check for sufficient space in outgoing buffer
    if (_uart->txspace() < frame_len) {
        debug("tx space too low");
        return false;
    }

    _uart->write(packet.bytes, frame_len);
    return true;
}

// convenience: send a frame carrying only a C2 (camera/feature) command,
// leaving the gimbal servo state unchanged (C1 = do-nothing)
bool AP_Mount_GimbalAI::send_c2_command(C2Package c2_pkg, uint8_t c2_instruction, const uint8_t *c2_params)
{
    return send_control_frame(SystemMode::DO_NOTHING, nullptr, c2_pkg, c2_instruction, c2_params);
}

// send target pitch and yaw angles to the pod
void AP_Mount_GimbalAI::send_target_angles(const MountAngleTarget &angle_rad)
{
    const float pitch_rad = angle_rad.pitch;
    const float yaw_rad = angle_rad.yaw;
    const bool yaw_is_ef = angle_rad.yaw_is_ef;

    // convert yaw to a body-frame angle (relative to the vehicle nose)
    float yaw_bf_rad = yaw_is_ef ? wrap_PI(yaw_rad - AP::ahrs().get_yaw_rad()) : yaw_rad;

    // enforce body-frame yaw angle limits
    yaw_bf_rad = constrain_float(yaw_bf_rad, radians(_params.yaw_angle_min), radians(_params.yaw_angle_max));

    // scale to the body-frame angle command (GIMF mode).  pitch is negated to
    // match the pod's joint-angle sign convention (negative = down)
    uint8_t params[10] {};
    put_le16(params, 0, (int16_t)(degrees(yaw_bf_rad) * AP_MOUNT_GIMBALAI_DEG_TO_OUTPUT));   // azimuth
    put_le16(params, 2, (int16_t)(-degrees(pitch_rad) * AP_MOUNT_GIMBALAI_DEG_TO_OUTPUT));   // pitch

    send_control_frame(SystemMode::ANGLE_FRAME, params, C2Package::NONE, 0, nullptr);
}

// send target pitch and yaw rates to the pod
void AP_Mount_GimbalAI::send_target_rates(const MountRateTarget &rate_rads)
{
    // scale rates to the velocity servo command (RATF mode, LSB = 0.01 deg/s).
    // pitch is negated to match the pod's joint-angle sign convention.
    uint8_t params[10] {};
    put_le16(params, 0, (int16_t)(degrees(rate_rads.yaw) * 100));    // azimuth rate
    put_le16(params, 2, (int16_t)(-degrees(rate_rads.pitch) * 100)); // pitch rate

    send_control_frame(SystemMode::RATE_SERVO, params, C2Package::NONE, 0, nullptr);
}

// take a picture.  returns true on success
bool AP_Mount_GimbalAI::take_picture()
{
    if (!_initialised) {
        return false;
    }
    uint8_t params[10] {};
    params[0] = 0x10;   // channel: all
    params[2] = 0x01;   // mode: single shot
    return send_c2_command(C2Package::ENCODING, (uint8_t)EncodingCmd::TAKE_PHOTO, params);
}

// start or stop video recording.  returns true on success
bool AP_Mount_GimbalAI::record_video(bool start_recording)
{
    if (!_initialised) {
        return false;
    }
    uint8_t params[10] {};
    params[0] = 0x10;                               // channel: all
    params[2] = start_recording ? 0x02 : 0x03;      // mode: start / stop continuous shooting
    return send_c2_command(C2Package::ENCODING, (uint8_t)EncodingCmd::TAKE_PHOTO, params);
}

// set zoom specified as a rate or percentage
bool AP_Mount_GimbalAI::set_zoom(ZoomType zoom_type, float zoom_value)
{
    if (!_initialised) {
        return false;
    }

    // zoom rate
    if (zoom_type == ZoomType::RATE) {
        VisibleCmd cmd = VisibleCmd::ZOOM_STOP;
        if (zoom_value < 0) {
            cmd = VisibleCmd::ZOOM_OUT;
        } else if (zoom_value > 0) {
            cmd = VisibleCmd::ZOOM_IN;
        }
        uint8_t params[10] {};
        put_le16(params, 0, 1); // default zoom speed
        return send_c2_command(C2Package::VISIBLE1, (uint8_t)cmd, params);
    }

    // zoom percentage -> electronic zoom integer multiple (0:X1 ... 15:X16)
    if (zoom_type == ZoomType::PCT) {
        const float zoom_times = linear_interpolate(1, AP_MOUNT_GIMBALAI_ZOOM_MAX, zoom_value, 0, 100);
        uint8_t params[10] {};
        put_le16(params, 0, (int16_t)constrain_int16((int16_t)roundf(zoom_times) - 1, 0, 15)); // integer multiple
        return send_c2_command(C2Package::VISIBLE1, (uint8_t)VisibleCmd::ELECTRONIC_ZOOM, params);
    }

    return false;
}

// set focus specified as rate, percentage or auto
// focus in = -1, focus hold = 0, focus out = 1
SetFocusResult AP_Mount_GimbalAI::set_focus(FocusType focus_type, float focus_value)
{
    if (!_initialised) {
        return SetFocusResult::FAILED;
    }

    switch (focus_type) {
    case FocusType::RATE: {
        VisibleCmd cmd = VisibleCmd::ZOOM_STOP;  // stops focus motion
        if (focus_value < 0) {
            cmd = VisibleCmd::FOCUS_MINUS;
        } else if (focus_value > 0) {
            cmd = VisibleCmd::FOCUS_PLUS;
        }
        if (!send_c2_command(C2Package::VISIBLE1, (uint8_t)cmd, nullptr)) {
            return SetFocusResult::FAILED;
        }
        return SetFocusResult::ACCEPTED;
    }
    case FocusType::PCT:
        return SetFocusResult::INVALID_PARAMETERS;
    case FocusType::AUTO:
        if (!send_c2_command(C2Package::VISIBLE1, (uint8_t)VisibleCmd::AUTO_FOCUS, nullptr)) {
            return SetFocusResult::FAILED;
        }
        return SetFocusResult::ACCEPTED;
    }

    return SetFocusResult::INVALID_PARAMETERS;
}

// set tracking to none, point or rectangle
// p1,p2 are in range 0 to 1.  0 is left or top, 1 is right or bottom
bool AP_Mount_GimbalAI::set_tracking(TrackingType tracking_type, const Vector2f& p1, const Vector2f& p2)
{
    if (!_initialised) {
        return false;
    }

    // image coordinates are pixels in a 1920x1080 frame, origin top-left
    static constexpr float IMG_WIDTH = 1920.0f;
    static constexpr float IMG_HEIGHT = 1080.0f;

    switch (tracking_type) {
    case TrackingType::TRK_NONE: {
        uint8_t params[10] {};
        put_le16(params, 0, 0); // automatic tracking off
        return send_c2_command(C2Package::AI, (uint8_t)AICmd::AUTO_TRACK, params);
    }
    case TrackingType::TRK_POINT: {
        uint8_t params[10] {};
        put_le16(params, 0, (int16_t)(p1.x * IMG_WIDTH));   // target centre azimuth (x) coordinate
        put_le16(params, 2, (int16_t)(p1.y * IMG_HEIGHT));  // target centre elevation (y) coordinate
        put_le16(params, 4, 0);                             // target id (0 = any)
        return send_control_frame(SystemMode::CLICK_TO_TRACK, params, C2Package::NONE, 0, nullptr);
    }
    case TrackingType::TRK_RECTANGLE: {
        const Vector2f centre = (p1 + p2) * 0.5f;
        const Vector2f size{fabsf(p2.x - p1.x), fabsf(p2.y - p1.y)};
        uint8_t params[10] {};
        put_le16(params, 0, (int16_t)(centre.x * IMG_WIDTH));   // target centre azimuth (x) coordinate
        put_le16(params, 2, (int16_t)(centre.y * IMG_HEIGHT));  // target centre pitch (y) coordinate
        put_le16(params, 4, (int16_t)(size.x * IMG_WIDTH));     // target width
        put_le16(params, 6, (int16_t)(size.y * IMG_HEIGHT));    // target height
        return send_control_frame(SystemMode::BOX_TRACK, params, C2Package::NONE, 0, nullptr);
    }
    }

    return false;
}

// set camera lens (image sensor channel) as a value from 0 to 4
bool AP_Mount_GimbalAI::set_lens(uint8_t lens)
{
    if (!_initialised) {
        return false;
    }
    // 0:visible1, 1:visible2, 2:infrared1, 3:infrared2, 4:image mosaic
    if (lens > 4) {
        return false;
    }
    uint8_t params[10] {};
    put_le16(params, 0, lens);  // channel number
    put_le16(params, 2, 1);     // channel switch on
    return send_c2_command(C2Package::IMAGE, (uint8_t)ImageCmd::CHANNEL_SWITCH, params);
}

// send camera settings message to GCS
void AP_Mount_GimbalAI::send_camera_settings(mavlink_channel_t chan) const
{
    if (!_initialised) {
        return;
    }

    // convert zoom times (1x ~ max) to zoom level (0 ~ 100)
    const float zoom_level = linear_interpolate(0, 100, _zoom_times, 1, AP_MOUNT_GIMBALAI_ZOOM_MAX);

    mavlink_msg_camera_settings_send(
        chan,
        AP_HAL::millis(),   // time_boot_ms
        _recording ? CAMERA_MODE_VIDEO : CAMERA_MODE_IMAGE, // camera mode
        zoom_level,         // zoomLevel 0 to 100
        NaNf);              // focusLevel unknown
}

// get rangefinder distance.  Returns true on success
bool AP_Mount_GimbalAI::get_rangefinder_distance(float& distance_m) const
{
    // healthy() check also covers the telemetry timeout that carries distance
    if (!healthy()) {
        return false;
    }
    distance_m = _rangefinder_dist_m;
    return true;
}

// enable/disable rangefinder.  Returns true on success
bool AP_Mount_GimbalAI::set_rangefinder_enable(bool enable)
{
    if (!_initialised) {
        return false;
    }
    if (enable) {
        uint8_t params[10] {};
        put_le16(params, 0, 0x05);  // 5Hz continuous ranging
        return send_c2_command(C2Package::LASER1, (uint8_t)LaserCmd::CONTINUOUS, params);
    }
    return send_c2_command(C2Package::LASER1, (uint8_t)LaserCmd::STOP, nullptr);
}

#endif // HAL_MOUNT_GIMBALAI_ENABLED
