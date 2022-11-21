#pragma once

/*
 * Driver the the NRA24 rangefinder
 * see http://en.nanoradar.cn/File/view/id/436.html
 */

#include "AP_RangeFinder_config.h"

#if AP_RANGEFINDER_NRA24_ENABLED

#include "AP_RangeFinder_Backend_Serial.h"

class AP_RangeFinder_NRA24 : public AP_RangeFinder_Backend_Serial
{

public:

    static AP_RangeFinder_Backend_Serial *create(
        RangeFinder::RangeFinder_State &_state,
        AP_RangeFinder_Params &_params) {
        return new AP_RangeFinder_NRA24(_state, _params);
    }

protected:

    // baudrate used during object construction:
    uint32_t initial_baudrate(uint8_t serial_instance) const override {
        return 115200;
    }

    // return sensor type as laser
    MAV_DISTANCE_SENSOR _get_mav_distance_sensor_type() const override {
        return MAV_DISTANCE_SENSOR_RADAR;
    }

    // get a reading, distance returned in reading_m
    bool get_reading(float &reading_m) override;

    // maximum time between readings before we change state to NoData:
    uint16_t read_timeout_ms() const override { return 500; }

private:

    using AP_RangeFinder_Backend_Serial::AP_RangeFinder_Backend_Serial;

    // parsing state
    enum class ParseState : uint8_t {
        WAITING_FOR_PREAMBLE1,
        WAITING_FOR_PREAMBLE2,
        WAITING_FOR_MESSAGEID_H,
        WAITING_FOR_MESSAGEID_L,
        WAITING_FOR_PAYLOAD,
        WAITING_FOR_CHECKSUM,
        WAITING_FOR_POSTAMBLE1,
        WAITING_FOR_POSTAMBLE2,
    } parse_state;

    // Process one byte received on serial port.  Returns true iif a
    // packet was parsed, in which case the distance is returned in
    // dist_cm
    bool parse_byte(uint8_t b, uint16_t &dist_cm);

    uint32_t last_distance_ms;                      // system time of last successful distance sensor read

    uint16_t message_id;
    uint8_t payload[7];
    uint8_t payload_bytes_received;

    uint8_t calculated_checksum;
};

#endif  // AP_RANGEFINDER_NRA24_ENABLED
