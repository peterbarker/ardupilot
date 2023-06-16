#pragma once

#include "AP_RangeFinder_config.h"

#if AP_RANGEFINDER_BENEWAKE_ENABLED

#include "AP_RangeFinder_Backend_Serial.h"
#include "AP_RangeFinder.h"

class AP_RangeFinder_Benewake : public AP_RangeFinder_Backend_Serial
{

public:

    using AP_RangeFinder_Backend_Serial::AP_RangeFinder_Backend_Serial;

protected:

    virtual MAV_DISTANCE_SENSOR _get_mav_distance_sensor_type() const override {
        return MAV_DISTANCE_SENSOR_LASER;
    }

    virtual float model_dist_max_cm() const = 0;
    virtual bool has_signal_byte() const { return false; }

private:

    // get a reading
    // distance returned in reading_m
    bool get_reading(float &reading_m) override;

    void move_preamble_in_buffer(uint8_t search_start_pos);

    union MsgUnion {
        struct {
            uint8_t header1;  // always x59
            uint8_t header2;  // always x59
            uint8_t dist_l; // low 8 bits (in cm)
            uint8_t dist_h; // high 8 bits (in cm)
            uint8_t strength_l;  // or reserved, depends on model
            uint8_t strength_h;  // or reserved, depends on model
            uint8_t sig_or_mode;  // depends on model (or reserved)
            uint8_t time;  // exposure time (or reserved)
            uint8_t checksum;
        };
        uint8_t linebuf[9];
        bool valid_checksum() const;
    } u;

    uint8_t linebuf_len;
};

#endif  // AP_RANGEFINDER_BENEWAKE_ENABLED
