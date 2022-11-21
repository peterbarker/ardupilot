#include "AP_RangeFinder_NRA24.h"

#if AP_RANGEFINDER_NRA24_ENABLED

#include <GCS_MAVLink/GCS.h>

bool AP_RangeFinder_NRA24::get_reading(float &reading_m)
{
    float sum_cm = 0;
    uint16_t count = 0;

    int16_t nbytes = MAX(uart->available(), 1024U);
    while (nbytes-- > 0) {
        int16_t r = uart->read();
        if (r < 0) {
            continue;
        }
        uint16_t dist_cm;
        if (parse_byte((uint8_t)r, dist_cm)) {
            sum_cm += dist_cm;
            count++;
        }
    }

    if (count == 0) {
        return false;
    }

    reading_m = sum_cm * 0.01f / count;

    last_distance_ms = AP_HAL::millis();

    return true;
}

bool AP_RangeFinder_NRA24::parse_byte(uint8_t byte, uint16_t &distance_cm)
{
    switch (parse_state) {
    case ParseState::WAITING_FOR_PREAMBLE1:
        if (byte == 0xAA) {
            parse_state = ParseState::WAITING_FOR_PREAMBLE2;
        }
        break;
    case ParseState::WAITING_FOR_PREAMBLE2:
        if (byte == 0xAA) {
            // good preamble
            parse_state = ParseState::WAITING_FOR_MESSAGEID_L;
        } else {
            // bad preamble
            parse_state = ParseState::WAITING_FOR_PREAMBLE1;
        }
        break;
    case ParseState::WAITING_FOR_MESSAGEID_L:
        message_id = byte;
        parse_state = ParseState::WAITING_FOR_MESSAGEID_H;
        break;
    case ParseState::WAITING_FOR_MESSAGEID_H:
        message_id |= byte << 8;
        calculated_checksum = 0;  // only covers payload
        payload_bytes_received = 0;
        parse_state = ParseState::WAITING_FOR_PAYLOAD;
        break;
    case ParseState::WAITING_FOR_PAYLOAD:
        payload[payload_bytes_received++] = byte;
        calculated_checksum += byte;
        if (payload_bytes_received == sizeof(payload)) {
            parse_state = ParseState::WAITING_FOR_CHECKSUM;
        }
        break;
    case ParseState::WAITING_FOR_CHECKSUM:
        if (byte != calculated_checksum) {
            // bad checksum
            parse_state = ParseState::WAITING_FOR_PREAMBLE1;
        } else {
            // good checksum
            parse_state = ParseState::WAITING_FOR_POSTAMBLE1;
        }
        break;
    case ParseState::WAITING_FOR_POSTAMBLE1:
        if (byte == 0x55) {
            // good postamble so far
            parse_state = ParseState::WAITING_FOR_POSTAMBLE2;
        } else {
            // bad postamble
            parse_state = ParseState::WAITING_FOR_PREAMBLE1;
        }
        break;
    case ParseState::WAITING_FOR_POSTAMBLE2:
        if (byte == 0x55) {
            // good postamble
            if (message_id != 0x70C) {
                // not a target_info message
                parse_state = ParseState::WAITING_FOR_PREAMBLE1;
                return false;
            }
            // extract distance from packet:
            distance_cm = payload[2] << 8 | payload[3];
            parse_state = ParseState::WAITING_FOR_PREAMBLE1;
            return true;
        }
        // bad postamble
        parse_state = ParseState::WAITING_FOR_PREAMBLE1;
        break;
    }
    return false;
}


#endif  // AP_RANGEFINDER_NRA24_ENABLED
