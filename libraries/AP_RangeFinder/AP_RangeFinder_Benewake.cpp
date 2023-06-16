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

#include "AP_RangeFinder_Benewake.h"

#if AP_RANGEFINDER_BENEWAKE_ENABLED

#include <AP_HAL/AP_HAL.h>
#include <AP_HAL/utility/sparse-endian.h>

#include <ctype.h>

extern const AP_HAL::HAL& hal;

#define BENEWAKE_FRAME_HEADER 0x59
#define BENEWAKE_FRAME_LENGTH 9
#define BENEWAKE_DIST_MAX_CM 32768
#define BENEWAKE_OUT_OF_RANGE_ADD 1.00  // metres

// format of serial packets received from benewake lidar
//
// Data Bit             Definition      Description
// ------------------------------------------------
// byte 0               Frame header    0x59
// byte 1               Frame header    0x59
// byte 2               DIST_L          Distance (in cm) low 8 bits
// byte 3               DIST_H          Distance (in cm) high 8 bits
// byte 4               STRENGTH_L      Strength low 8 bits
// bute 4 (TF03)        (Reserved)
// byte 5               STRENGTH_H      Strength high 8 bits
// bute 5 (TF03)        (Reserved)
// byte 6 (TF02)        SIG             Reliability in 8 levels, 7 & 8 means reliable
// byte 6 (TFmini)      Distance Mode   0x02 for short distance (mm), 0x07 for long distance (cm)
// byte 6 (TF03)        (Reserved)
// byte 7 (TF02 only)   TIME            Exposure time in two levels 0x03 and 0x06
// byte 8               Checksum        Checksum byte, sum of bytes 0 to bytes 7

// distance returned in reading_m, signal_ok is set to true if sensor reports a strong signal

void AP_RangeFinder_Benewake::move_preamble_in_buffer(uint8_t search_start_pos)
{
    uint8_t i;
    for (i=search_start_pos; i<linebuf_len; i++) {
        if ((uint8_t)u.linebuf[i] == BENEWAKE_FRAME_HEADER) {
            break;
        }
    }
    if (i == 0) {
        return;
    }
    memmove(u.linebuf, &u.linebuf[i], linebuf_len-i);
    linebuf_len -= i;
}

bool AP_RangeFinder_Benewake::MsgUnion::valid_checksum() const
{
    uint8_t ret = 0;
    for (uint8_t i=0; i<8; i++) {
        ret += linebuf[i];
    }
    return ret;
}

bool AP_RangeFinder_Benewake::get_reading(float &reading_m)
{
    if (uart == nullptr) {
        return false;
    }

    float sum_cm = 0;
    uint16_t count = 0;
    uint16_t count_out_of_range = 0;

    uint32_t nbytes = MIN(uart->available(), 4096);

    while (nbytes > 0) {
        const auto bytes_to_read = MIN(nbytes, ARRAY_SIZE(u.linebuf)-linebuf_len);
        const uint32_t nread = uart->read(&u.linebuf[linebuf_len], bytes_to_read);
        if (nread == 0) {
            break;
        }
        nbytes -= nread;
        linebuf_len += nread;

        move_preamble_in_buffer(0);

        if (linebuf_len < BENEWAKE_FRAME_LENGTH) {
            continue;
        }

        if (u.linebuf[1] != BENEWAKE_FRAME_HEADER) {
            move_preamble_in_buffer(1);
            continue;
        }

        if (!u.valid_checksum()) {
            move_preamble_in_buffer(1);
            continue;
        }

        // calculate distance
        const uint16_t dist = u.dist_h << 8 | u.dist_l;
        if (dist >= BENEWAKE_DIST_MAX_CM || dist == uint16_t(model_dist_max_cm())) {
            // this reading is out of range. Note that we
            // consider getting exactly the model dist max
            // is out of range. This fixes an issue with
            // the TF03 which can give exactly 18000 cm
            // when out of range
            count_out_of_range++;
            continue;
        }
        if (has_signal_byte()) {
            // TF02 provides signal reliability (good = 7 or 8)
            if (u.sig_or_mode < 7) {
                // this reading is out of range
                count_out_of_range++;
                continue;
            }
        }
        // add distance to sum
        sum_cm += dist;
        count++;

        // clear buffer - this only works because
        // ARRAY_SIZE(LINEBUF_LEN) is the length of a frame
        linebuf_len = 0;
    }

    if (count > 0) {
        // return average distance of readings
        reading_m = (sum_cm * 0.01f) / count;
        return true;
    }

    if (count_out_of_range > 0) {
        // if only out of range readings return larger of
        // driver defined maximum range for the model and user defined max range + 1m
        reading_m = MAX(model_dist_max_cm() * 0.01, max_distance() + BENEWAKE_OUT_OF_RANGE_ADD);
        return true;
    }

    // no readings so return false
    return false;
}

#endif  // AP_RANGEFINDER_BENEWAKE_ENABLED
