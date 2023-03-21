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
  support for serial connected AHRS systems
 */

#define ALLOW_DOUBLE_MATH_FUNCTIONS

#include "AP_ExternalAHRS_AdvancedNavigation.h"
#include <AP_Math/AP_Math.h>
#include <AP_Baro/AP_Baro.h>
#include <AP_Compass/AP_Compass.h>
#include <AP_GPS/AP_GPS.h>
#include <AP_InertialSensor/AP_InertialSensor.h>
#include <GCS_MAVLink/GCS.h>
#include <AP_Logger/AP_Logger.h>
#include <AP_Common/NMEA.h>
#include <stdio.h>
#include <AP_BoardConfig/AP_BoardConfig.h>


#define AN_PACKET_ID_PACKET_PERIODS 181
#define AN_PACKET_ID_SATELLITES 30
#define AN_PACKET_ID_RAW_GNSS 29
#define AN_PACKET_ID_RAW_SENSORS 28
#define AN_PACKET_ID_VELOCITY_STANDARD_DEVIATION 25
#define AN_PACKET_ID_SYSTEM_STATE 20
#define AN_PACKET_ID_DEVICE_INFO 3
#define AN_PACKET_ID_ACKNOWLEDGE 0
#define AN_GPS_EPOCH_UNIX_OFFSET 315964800 // GPS Week 0 sec 0 is midnight Sunday Jan 6th 1980 UTC
#define AN_TIMEOUT 5000 //ms
#define AN_MAXIMUM_PACKET_PERIODS 50

#define an_packet_pointer(packet) packet->header
#define an_packet_size(packet) (packet->length + AN_PACKET_HEADER_SIZE)*sizeof(uint8_t)
#define an_packet_crc(packet) ((packet->header[4]<<8) | packet->header[3])

#if HAL_EXTERNAL_AHRS_ENABLED

extern const AP_HAL::HAL &hal;

/*
    Packet for requesting a Advanced Navigation Device to send its
    device information (ANPP Packet 3) a single time.
        0x9a-0xd1   - Header see ANPP Documentation for more info
        0x03        - Request Packet 3 (Device Info Packet)
*/
static const uint8_t request_an_info[] {0x9a, 0x01, 0x01, 0x93, 0xd1, 0x03};

/*
    Packet for requesting a Advanced Navigation Device to set its
    Packet Timer Period to 1000Hz
        0xa0-0x6c   - Header see ANPP Documentation for more info
        0x01        - True - Permanent effect
        0x01        - True - UTC Sync
        0xe8,0x03   - Packet Timer Period ms (uint16_t) (1000Hz)
*/
static const uint8_t timer_period_1000_hz[] {0xa0, 0xb4, 0x04, 0x3c, 0x6c, 0x01, 0x01, 0xe8, 0x03};


/*
 * Function to decode ANPP Packets from raw buffer in the decoder structure
 * Returns false for a buffer error
 */
int AP_ExternalAHRS_AdvancedNavigation_Decoder::decode_packet(uint8_t* out_buffer, size_t buf_size)
{
    uint16_t decode_iterator = 0;
    uint8_t header_lrc, length;
    uint16_t crc;

    // Iterate through buffer until no more headers could be in buffer
    while (decode_iterator + AN_PACKET_HEADER_SIZE <= _buffer_length) {
        header_lrc = _buffer[decode_iterator++];

        // Is this the start of a valid header?
        if (header_lrc == calculate_header_lrc(&_buffer[decode_iterator])) {
            decode_iterator++; // skip ID as it is unused (-Werror=unused-but-set-variable)
            length = _buffer[decode_iterator++];
            crc = _buffer[decode_iterator++];
            crc |= _buffer[decode_iterator++] << 8;

            // If the packet length is over the edge of the buffer
            if (decode_iterator + length > _buffer_length) {
                decode_iterator -= AN_PACKET_HEADER_SIZE;
                break;
            }

            // If the crc matches then a valid packet has been identified.
            if (crc == crc16_ccitt(&_buffer[decode_iterator], length, 0xFFFF)) {

                // Protect from buffer overflow.
                if ((size_t) (length + AN_PACKET_HEADER_SIZE) > buf_size) {
                    return false;
                }

                // Save the data into the output buffer.
                memcpy(out_buffer, &_buffer[decode_iterator - AN_PACKET_HEADER_SIZE], AN_PACKET_HEADER_SIZE + length * sizeof(uint8_t));

                decode_iterator += length;
                _packets_decoded++;
                _bytes_decoded += length + AN_PACKET_HEADER_SIZE;
                break;
            } else { // Invalid packet for given header
                decode_iterator -= (AN_PACKET_HEADER_SIZE - 1);
                _crc_errors++;
                _bytes_discarded++;
            }
        } else { // Invalid Header
            _lrc_errors++;
            _bytes_discarded++;
        }
    }
    // If there is still buffer to be decoded.
    if (decode_iterator < _buffer_length) {
        // Ensure that the iterator isn't invalid
        if (decode_iterator > 0) {
            // move the unparsed memory to the beginning of the buffer.
            memmove(&_buffer[0], &_buffer[decode_iterator], (_buffer_length - decode_iterator) * sizeof(uint8_t));
            _buffer_length -= decode_iterator;
            _complete = false;
            return true;
        }
    } else {
        _buffer_length = 0;
    }

    _complete = true;
    return true;
}


// constructor
AP_ExternalAHRS_AdvancedNavigation::AP_ExternalAHRS_AdvancedNavigation(AP_ExternalAHRS *_frontend,
        AP_ExternalAHRS::state_t &_state) :
    AP_ExternalAHRS_backend(_frontend, _state)
{
    auto &sm = AP::serialmanager();
    _uart = sm.find_serial(AP_SerialManager::SerialProtocol_AHRS, 0);
    if (_uart == nullptr) {
        GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "ExternalAHRS no UART");
        return;
    }

    _baudrate = sm.find_baudrate(AP_SerialManager::SerialProtocol_AHRS, 0);
    _port_num = sm.find_portnum(AP_SerialManager::SerialProtocol_AHRS, 0);

    _last_vel_sd = new AN_VELOCITY_STANDARD_DEVIATION;
    _last_satellites = new AN_SATELLITES;

    if (!hal.scheduler->thread_create(FUNCTOR_BIND_MEMBER(&AP_ExternalAHRS_AdvancedNavigation::update_thread, void), "AHRS", 2048, AP_HAL::Scheduler::PRIORITY_SPI, 0)) {
        AP_HAL::panic("Failed to start ExternalAHRS update thread");
    }

    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "ExternalAHRS initialised");
}

void AP_ExternalAHRS_AdvancedNavigation::update_thread(void)
{
    if (!_port_open) {
        _uart->begin(_baudrate);
        _port_open = true;
    }

    while (true) {
        // Request data. If error occurs notify.
        if (!request_data()) {
            GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "ExternalAHRS: Request Data Error");
        }

        // Sleep the scheduler
        hal.scheduler->delay_microseconds(1000);

        // Collect the requested packets from the UART manager
        if (!get_packets()) {
            GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "ExternalAHRS: Error Receiving Packets");
        }
    }
}

bool AP_ExternalAHRS_AdvancedNavigation::get_packets(void)
{
    if (!_port_open) {
        return false;
    }

    {
        // Ensure that this section is completed in a single thread.
        WITH_SEMAPHORE(_sem);

        // guard for no data on uart.
        if (_uart->available() <= 0) {
            return true;
        }

        // receive packets from the UART into the decoder
        _decoder.receive(_uart->read(_decoder.pointer(), _decoder.size()));

        if (_decoder.bytes_received() > 0) {
            // Decode all packets in the buffer
            while (!_decoder.is_complete()) {
                // decode a packet into the message buffer
                if (!_decoder.decode_packet(_msg.buffer, sizeof(_msg.buffer))) {
                    return false;
                }
                handle_packet();
            }
        }
    }
    return true;
}

bool AP_ExternalAHRS_AdvancedNavigation::request_data(void)
{
    if (!_port_open) {
        return false;
    }

    // Update device info every 20 secs
    if ((AP_HAL::millis() - _last_device_info_pkt_ms > 20000) || (_last_device_info_pkt_ms == 0)) {
        if (_uart->txspace() < sizeof(request_an_info)) {
            return false;
        }
        _uart->write(request_an_info, sizeof(request_an_info));
    }

    // Don't send a packet request unless the device is healthy
    if (_current_rate != get_rate() && healthy()) {
        if (!sendPacketRequest()) {
            GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "ExternalAHRS: Failure to send packet request");
        }
    }

    return true;
}

int8_t AP_ExternalAHRS_AdvancedNavigation::get_port(void) const
{
    if (!_uart) {
        return -1;
    }
    return _port_num;
};

// Get model/type name
const char* AP_ExternalAHRS_AdvancedNavigation::get_name() const
{
    if ((AP_HAL::millis() - _last_pkt_ms) > AN_TIMEOUT) {
        static char buf[30];
        hal.util->snprintf(buf, 30, "AdNav: TIMEOUT... %8ums", (unsigned int) (AP_HAL::millis() - _last_pkt_ms));
        return buf;
    }

    if (_last_device_info_pkt_ms == 0) {
        return "AdNav No Connection...";
    }

    switch (_device_id) {
    case 0:
        return "Uninitialized Device ID";
        break;
    case device_id_spatial:
        return "AdNav Spatial";
        break;
    case device_id_orientus:
    case device_id_orientus_v3:
        return "AdNav Orientus";
        break;
    case device_id_spatial_fog:
        return "AdNav Spatial FOG";
        break;
    case device_id_spatial_dual:
        return "AdNav Spatial Dual";
        break;
    case device_id_ilu:
        return "AdNav Interface Logging Unit";
        break;
    case device_id_air_data_unit:
        return "AdNav Air Data Unit";
        break;
    case device_id_spatial_fog_dual:
        return "AdNav Spatial FOG Dual";
        break;
    case device_id_motus:
        return "AdNav Motus";
        break;
    case device_id_gnss_compass:
        return "AdNav GNSS Compass";
        break;
    case device_id_certus:
        return "AdNav Certus";
        break;
    case device_id_aries:
        return "AdNav Aries";
        break;
    case device_id_boreas_d90:
    case device_id_boreas_d90_fpga:
    case device_id_boreas_coil:
        return "AdNav Boreas";
        break;
    case device_id_certus_mini_a:
        return "AdNav Certus Mini A";
        break;
    case device_id_certus_mini_n:
        return "AdNav Certus Mini N";
        break;
    case device_id_certus_mini_d:
        return "AdNav Certus Mini D";
        break;
    default:
        return "Unknown AdNav Device ID";
    }
}

bool AP_ExternalAHRS_AdvancedNavigation::healthy(void) const
{
    uint32_t now = AP_HAL::millis();
    return ((now - _last_state_pkt_ms) < 10000);
}

bool AP_ExternalAHRS_AdvancedNavigation::initialised(void) const
{
    if (get_gnss_capability()) {
        return _last_state_pkt_ms != 0 && _last_device_info_pkt_ms != 0 && _last_raw_gnss_pkt_ms !=0;
    } else {
        return _last_state_pkt_ms != 0 && _last_device_info_pkt_ms != 0;
    }
}

bool AP_ExternalAHRS_AdvancedNavigation::pre_arm_check(char *failure_msg, uint8_t failure_msg_len) const
{
    bool hit = true;

    // Get the last status flags and put them into readable bitfield structure.
    AN_SYSTEM_STATE state_flags{
        system_status: {
            r: _system_status
        },
        filter_status: {
            r: _filter_status
        },
    };

    // Add failure messages
    if (AP_HAL::millis() - _last_pkt_ms > AN_TIMEOUT) {
        hal.util->snprintf(failure_msg, failure_msg_len, "DEVICE TIMEOUT Last Packet %8ums ago", (unsigned int) (AP_HAL::millis() - _last_pkt_ms));
        hit = true;
        return false;
    }
    if (!healthy()) {
        hal.util->snprintf(failure_msg, failure_msg_len, "%sDevice unhealthy, ", failure_msg);
        hit = false;
    }
    if ((state_flags.filter_status.b.gnss_fix_type < 1) && (get_gnss_capability())) {
        hal.util->snprintf(failure_msg, failure_msg_len, "%sNo GPS lock, ", failure_msg);
        hit = false;
    }
    if (state_flags.system_status.b.system_failure) {
        hal.util->snprintf(failure_msg, failure_msg_len, "%sSystem Failure, ", failure_msg);
        hit = false;
    }
    if (state_flags.system_status.b.accelerometer_sensor_failure) {
        hal.util->snprintf(failure_msg, failure_msg_len, "%sAccelerometer Failure, ", failure_msg);
        hit = false;
    }
    if (state_flags.system_status.b.gyroscope_sensor_failure) {
        hal.util->snprintf(failure_msg, failure_msg_len, "%sGyroscope Failure, ", failure_msg);
        hit = false;
    }
    if (state_flags.system_status.b.magnetometer_sensor_failure) {
        hal.util->snprintf(failure_msg, failure_msg_len, "%sMagnetometer Failure, ", failure_msg);
        hit = false;
    }
    if (state_flags.system_status.b.pressure_sensor_failure) {
        hal.util->snprintf(failure_msg, failure_msg_len, "%sBarometer Failure, ", failure_msg);
        hit = false;
    }
    if (state_flags.system_status.b.gnss_failure) {
        hal.util->snprintf(failure_msg, failure_msg_len, "%sGNSS Failure, ", failure_msg);
        hit = false;
    }
    if (!state_flags.filter_status.b.orientation_filter_initialised) {
        hal.util->snprintf(failure_msg, failure_msg_len, "%sOrientation Filter Not Initialised, ", failure_msg);
        hit = false;
    }
    if (!state_flags.filter_status.b.ins_filter_initialised) {
        hal.util->snprintf(failure_msg, failure_msg_len, "%sINS Filter Not Initialised, ", failure_msg);
        hit = false;
    }
    if (!state_flags.filter_status.b.heading_initialised) {
        hal.util->snprintf(failure_msg, failure_msg_len, "%sHeading Filter Not Initialised, ", failure_msg);
        hit = false;
    }
    return hit;
}

void AP_ExternalAHRS_AdvancedNavigation::get_filter_status(nav_filter_status &status) const
{
    // Get the last filter status flags and put them into readable bitfield structure.
    AN_SYSTEM_STATE state_flags{
        system_status: {
            r: 0
        },
        filter_status: {
            r: _filter_status
        }
    };

    memset(&status, 0, sizeof(status));
    if (_last_state_pkt_ms != 0) {
        status.flags.initalized = 1;
    }
    if (healthy() && _last_state_pkt_ms != 0) {
        status.flags.attitude = 1;
        if (get_gnss_capability()) {
            status.flags.vert_vel = 1;
            status.flags.vert_pos = 1;

            if (state_flags.filter_status.b.gnss_fix_type > gnss_fix_none) {
                status.flags.horiz_vel = 1;
                status.flags.horiz_pos_rel = 1;
                status.flags.horiz_pos_abs = 1;
                status.flags.pred_horiz_pos_rel = 1;
                status.flags.pred_horiz_pos_abs = 1;
                status.flags.using_gps = 1;
            }

            if (state_flags.filter_status.b.gnss_fix_type > gnss_fix_2d) {
                status.flags.gps_quality_good = 1;
                status.flags.vert_pos = 1;
                status.flags.vert_vel = 1;
            }
        }
    }

}

void AP_ExternalAHRS_AdvancedNavigation::send_status_report(GCS_MAVLINK &link) const
{
    // prepare flags
    uint16_t flags = 0;
    nav_filter_status filterStatus;
    get_filter_status(filterStatus);
    if (filterStatus.flags.attitude) {
        flags |= EKF_ATTITUDE;
    }
    if (filterStatus.flags.horiz_vel) {
        flags |= EKF_VELOCITY_HORIZ;
    }
    if (filterStatus.flags.vert_vel) {
        flags |= EKF_VELOCITY_VERT;
    }
    if (filterStatus.flags.horiz_pos_rel) {
        flags |= EKF_POS_HORIZ_REL;
    }
    if (filterStatus.flags.horiz_pos_abs) {
        flags |= EKF_POS_HORIZ_ABS;
    }
    if (filterStatus.flags.vert_pos) {
        flags |= EKF_POS_VERT_ABS;
    }
    if (filterStatus.flags.terrain_alt) {
        flags |= EKF_POS_VERT_AGL;
    }
    if (filterStatus.flags.const_pos_mode) {
        flags |= EKF_CONST_POS_MODE;
    }
    if (filterStatus.flags.pred_horiz_pos_rel) {
        flags |= EKF_PRED_POS_HORIZ_REL;
    }
    if (filterStatus.flags.pred_horiz_pos_abs) {
        flags |= EKF_PRED_POS_HORIZ_ABS;
    }
    if (!filterStatus.flags.initalized) {
        flags |= EKF_UNINITIALIZED;
    }

    // send message
    const float vel_gate = 5; // represents hz value data is posted at
    const float pos_gate = 5; // represents hz value data is posted at
    const float hgt_gate = 5; // represents hz value data is posted at
    const float mag_var =  5;
    mavlink_msg_ekf_status_report_send(link.get_chan(), flags,
                                       norm(_last_vel_sd->x,
                                            _last_vel_sd->y,
                                            _last_vel_sd->z)/vel_gate,
                                       norm(_gnss_sd[0],
                                            _gnss_sd[1])/pos_gate,
                                       _gnss_sd[2]/hgt_gate,
                                       mag_var,
                                       0,
                                       0);
}

/*
 * Function to request data packets from a Advanced Navigation Device at the current rate.
 */
bool AP_ExternalAHRS_AdvancedNavigation::sendPacketRequest()
{
    // Set the device to use a period timer of 1000Hz
    // See ANPP Packet Rates for more info
    if (_uart->txspace() < sizeof(timer_period_1000_hz)) {
        return false;
    }
    _uart->write(timer_period_1000_hz, sizeof(timer_period_1000_hz));

    // Update the current rate
    _current_rate = get_rate();

    AN_PACKETS_PERIOD periods{
        permanent: true,
        clear_existing_packet_periods: true,
        periods: {
            AN_PERIOD{
                id: AN_PACKET_ID_SYSTEM_STATE,
                packet_period: (uint32_t) 1.0e3 / _current_rate
            },
            AN_PERIOD{
                id: AN_PACKET_ID_VELOCITY_STANDARD_DEVIATION,
                packet_period: (uint32_t) 1.0e3 / _current_rate
            },
            AN_PERIOD{
                id: AN_PACKET_ID_RAW_SENSORS,
                packet_period: (uint32_t) 1.0e3 / _current_rate
            },
            AN_PERIOD{
                id: AN_PACKET_ID_RAW_GNSS,
                packet_period: (uint32_t) 1.0e3 / _current_rate
            },
            AN_PERIOD{
                id: AN_PACKET_ID_SATELLITES,
                packet_period: (uint32_t) 1.0e3 / _current_rate
            }
        }
    };

    AN_PACKET packet;
    // load the AN_PACKETS_PERIOD Into the payload.
    packet.payload.packets_period = periods;
    packet.update_checks(AN_PACKET_ID_PACKET_PERIODS, (5*sizeof(AN_PERIOD)) + 2);

    // Check for space in the tx buffer
    if (_uart->txspace() < packet.packet_size()) {
        return false;
    }
    _uart->write(packet.raw_pointer(), packet.packet_size());

    return true;
}

/*
 * Function that returns the gps capability of the connected AdNav device.
 */
bool AP_ExternalAHRS_AdvancedNavigation::get_gnss_capability(void) const
{
    switch (_device_id) {
    case device_id_orientus:
    case device_id_orientus_v3:
    case device_id_air_data_unit:
    case device_id_motus:
    case device_id_certus_mini_a:
        return false;
        break;
    default:
        return true;
        break;
    }
}

/*
 * Function that returns the barometric capability of the connected AdNav device.
 */
bool AP_ExternalAHRS_AdvancedNavigation::get_baro_capability(void) const
{
    switch (_device_id) {
    case device_id_air_data_unit:
    case device_id_orientus:
    case device_id_orientus_v3:
    case device_id_gnss_compass:
    case device_id_certus_mini_a:
        return false;
        break;
    case device_id_motus:
        // Motus versions prior to 2.3 didn't have a barometer enabled.
        if (_hardware_rev < 2300) {
            return false;
        }
        break;
    default:
        break;
    }
    return true;
}

void AP_ExternalAHRS_AdvancedNavigation::handle_packet()
{
    // get current time
    uint32_t now = AP_HAL::millis();
    _last_pkt_ms = now;

    // Update depending on received packet.
    switch (_msg.packet.id) {
    case AN_PACKET_ID_DEVICE_INFO: {
        _last_device_info_pkt_ms = now;
        _device_id = _msg.packet.payload.device_info.device_id;
        _hardware_rev = _msg.packet.payload.device_info.hardware_revision;
        break;
    }
    case AN_PACKET_ID_SYSTEM_STATE: {
        _last_state_pkt_ms = now;
        _system_status = _msg.packet.payload.system_state.system_status.r;
        _filter_status = _msg.packet.payload.system_state.filter_status.r;
        {
            WITH_SEMAPHORE(state.sem);

            state.accel = Vector3f{
                _msg.packet.payload.system_state.body_acceleration_x,
                _msg.packet.payload.system_state.body_acceleration_y,
                _msg.packet.payload.system_state.body_acceleration_z
            };
            state.gyro = Vector3f{
                _msg.packet.payload.system_state.angular_velocity_x,
                _msg.packet.payload.system_state.angular_velocity_y,
                _msg.packet.payload.system_state.angular_velocity_z
            };

            state.have_velocity = true;
            state.velocity = Vector3f{
                _msg.packet.payload.system_state.velocity_n,
                _msg.packet.payload.system_state.velocity_e,
                _msg.packet.payload.system_state.velocity_d
            };

            if (get_gnss_capability()) {
                state.have_location = true;
                state.location = Location{
                    (int32_t) (degrees(_msg.packet.payload.system_state.latitude) * 1.0e7),
                    (int32_t) (degrees(_msg.packet.payload.system_state.longitude) * 1.0e7),
                    (int32_t) (_msg.packet.payload.system_state.height *1.0e2),
                    Location::AltFrame::ABSOLUTE
                };
            }

            state.have_quaternion = true;
            state.quat.from_euler(
                _msg.packet.payload.system_state.roll,
                _msg.packet.payload.system_state.pitch,
                _msg.packet.payload.system_state.heading
            );
        }
        break;
    }
    case AN_PACKET_ID_VELOCITY_STANDARD_DEVIATION: {
        // save packet to be used for external gps.
        *_last_vel_sd = _msg.packet.payload.velocity_standard_deviation;
        break;
    }

    case AN_PACKET_ID_RAW_SENSORS: {
        AP_ExternalAHRS::ins_data_message_t ins{
            accel: Vector3f{
                _msg.packet.payload.raw_sensors.accelerometer_x,
                _msg.packet.payload.raw_sensors.accelerometer_y,
                _msg.packet.payload.raw_sensors.accelerometer_z
            },
            gyro: Vector3f{
                _msg.packet.payload.raw_sensors.gyroscope_x,
                _msg.packet.payload.raw_sensors.gyroscope_y,
                _msg.packet.payload.raw_sensors.gyroscope_z
            },
            temperature: _msg.packet.payload.raw_sensors.imu_temperature
        };
        AP::ins().handle_external(ins);


#if AP_COMPASS_EXTERNALAHRS_ENABLED
        AP_ExternalAHRS::mag_data_message_t mag {
            field: Vector3f{
                _msg.packet.payload.raw_sensors.magnetometer_x,
                _msg.packet.payload.raw_sensors.magnetometer_y,
                _msg.packet.payload.raw_sensors.magnetometer_z,
            }
        };
        AP::compass().handle_external(mag);
#endif
#if AP_BARO_EXTERNALAHRS_ENABLED
        if (get_baro_capability()) {
            AP_ExternalAHRS::baro_data_message_t baro{
                instance: 0,
            pressure_pa: _msg.packet.payload.raw_sensors.pressure,
            temperature: _msg.packet.payload.raw_sensors.pressure_temperature
            };
            AP::baro().handle_external(baro);
        };
#endif
    }
    break;

    case AN_PACKET_ID_RAW_GNSS: {
        // Save the standard deviations for status report
        _gnss_sd[0] = _msg.packet.payload.raw_gnss.lat_sd;
        _gnss_sd[1] = _msg.packet.payload.raw_gnss.long_sd;
        _gnss_sd[2] = _msg.packet.payload.raw_gnss.height_sd;

        AP_ExternalAHRS::gps_data_message_t gps;

        uint32_t sinceLastEpoch = _msg.packet.payload.raw_gnss.unix_time - AN_GPS_EPOCH_UNIX_OFFSET;
        uint8_t fix = 0;

        switch (_msg.packet.payload.raw_gnss.flags.b.fix_type) {
        case gnss_fix_none:
            fix = GPS_FIX_TYPE_NO_FIX;
            break;
        case gnss_fix_2d:
            fix = GPS_FIX_TYPE_2D_FIX;
            break;
        case gnss_fix_3d:
            fix = GPS_FIX_TYPE_3D_FIX;
            break;
        case gnss_fix_sbas:
        case gnss_fix_differential:
            fix = GPS_FIX_TYPE_DGPS;
            break;
        case gnss_fix_omnistar:
            fix = GPS_FIX_TYPE_PPP;
            break;
        case gnss_fix_rtk_float:
            fix = GPS_FIX_TYPE_RTK_FLOAT;
            break;
        case gnss_fix_rtk_fixed:
            fix = GPS_FIX_TYPE_RTK_FIXED;
            break;
        default:
            break;
        }

        gps.gps_week = (uint16_t) floor(sinceLastEpoch / AP_SEC_PER_WEEK);
        gps.ms_tow = (uint32_t) (((sinceLastEpoch - (AP_SEC_PER_WEEK * gps.gps_week)) * 1.0e3) + (_msg.packet.payload.raw_gnss.unix_microseconds * 1.0e-3));
        gps.fix_type = fix;
        gps.satellites_in_view = (uint8_t) (_last_satellites->beidou_satellites + _last_satellites->galileo_satellites
                                            + _last_satellites->glonass_satellites + _last_satellites->gps_satellites + _last_satellites->sbas_satellites);

        gps.horizontal_pos_accuracy = (float) norm(_msg.packet.payload.raw_gnss.lat_sd, _msg.packet.payload.raw_gnss.long_sd);
        gps.vertical_pos_accuracy = _msg.packet.payload.raw_gnss.height_sd;

        gps.hdop = _last_satellites->hdop;
        gps.vdop = _last_satellites->vdop;

        gps.longitude = (int32_t) (degrees(_msg.packet.payload.raw_gnss.longitude) * 1.0e7);
        gps.latitude = (int32_t) (degrees(_msg.packet.payload.raw_gnss.latitude) * 1.0e7);
        gps.msl_altitude = (int32_t) (_msg.packet.payload.raw_gnss.height * 1.0e2);

        gps.ned_vel_north = _msg.packet.payload.raw_gnss.velocity_n;
        gps.ned_vel_east = _msg.packet.payload.raw_gnss.velocity_e;
        gps.ned_vel_down = _msg.packet.payload.raw_gnss.velocity_d;

        gps.horizontal_vel_accuracy = (float) norm(
                                          _last_vel_sd->x,
                                          _last_vel_sd->y,
                                          _last_vel_sd->z
                                      );

        AP::gps().handle_external(gps);
    }
    break;

    case AN_PACKET_ID_SATELLITES: {
        // save packet to be used for external gps.
        *_last_satellites = _msg.packet.payload.satellites;
    }
    break;

    default: {

    }
    break;
    }
}

#endif  // HAL_EXTERNAL_AHRS_ENABLED

