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
  Simulator for Topotek gimbal
*/

#include "SIM_config.h"

#if AP_SIM_TOPOTEK_ENABLED

#include "SIM_Topotek.h"
#include <AP_Math/AP_Math.h>
#include <errno.h>

using namespace SITL;

void Topotek::update(const Aircraft &aircraft)
{
    gimbal.update(aircraft);
    update_input();

    // send attitude at 10 Hz
    const uint32_t now_ms = AP_HAL::millis();
    if (now_ms - _last_attitude_ms >= 100) {
        _last_attitude_ms = now_ms;
        send_attitude();
    }
}

void Topotek::send_attitude()
{
    Vector3f joint_angles;
    gimbal.get_joint_angles(joint_angles);

    // driver negates pitch on receipt (pitch_angle_cd = -hexchar4_to_int16(...)),
    // so send negative of the gimbal's pitch to produce the correct sign at the driver
    const int16_t yaw_cd   = (int16_t)(degrees(joint_angles.z) * 100.0f);
    const int16_t pitch_cd = (int16_t)(-degrees(joint_angles.y) * 100.0f);
    const int16_t roll_cd  = (int16_t)(degrees(joint_angles.x) * 100.0f);

    uint8_t data[12];
    uint16_to_hex4((uint16_t)yaw_cd,   &data[0]);
    uint16_to_hex4((uint16_t)pitch_cd, &data[4]);
    uint16_to_hex4((uint16_t)roll_cd,  &data[8]);

    send_packet('G', "GIA", false, data, sizeof(data));
}

/*
  read bytes from autopilot into _buf, then scan for complete packets.
  Packet format:
    [0]    '#'
    [1]    'T' or 't'
    [2]    'P' or 'p'
    [3]    'U' (addr1)
    [4]    addr2 ('G','D','M','E','P')
    [5]    data_len as a single ASCII hex char
    [6]    'r' or 'w'
    [7..9] 3-char command ID
    [10..10+data_len-1]  data bytes
    [10+data_len..11+data_len]  2-byte CRC
  Total packet length = 12 + data_len
*/
void Topotek::update_input()
{
    const ssize_t n = read_from_autopilot((char*)&_buf[_buflen], ARRAY_SIZE(_buf) - _buflen - 1);
    if (n < 0) {
        if (errno != EAGAIN && errno != EWOULDBLOCK && errno != 0) {
            AP_HAL::panic("Failed to read from autopilot");
        }
        return;
    }
    _buflen += n;

    while (_buflen >= 3) {
        // search for '#' at the start
        if (_buf[0] != '#') {
            memmove(_buf, &_buf[1], --_buflen);
            continue;
        }
        if (_buf[1] != 'T' && _buf[1] != 't') {
            memmove(_buf, &_buf[1], --_buflen);
            continue;
        }
        if (_buf[2] != 'P' && _buf[2] != 'p') {
            memmove(_buf, &_buf[1], --_buflen);
            continue;
        }

        // need at least 6 bytes to read the data_len field
        if (_buflen < 6) {
            break;
        }

        // parse data length from ASCII hex char at [5]
        const uint8_t data_len = char_to_hex(_buf[5]);
        if (data_len == 255) {
            // invalid data length — discard '#'
            memmove(_buf, &_buf[1], --_buflen);
            continue;
        }

        const uint8_t pkt_len = 12 + data_len;
        if (pkt_len > PACKETLEN_MAX) {
            memmove(_buf, &_buf[1], --_buflen);
            continue;
        }

        // wait for the full packet
        if (_buflen < pkt_len) {
            break;
        }

        // verify and dispatch the packet
        handle_packet(data_len);

        // consume packet bytes regardless of CRC outcome
        memmove(_buf, &_buf[pkt_len], _buflen - pkt_len);
        _buflen -= pkt_len;
    }
}

void Topotek::handle_packet(uint8_t data_len)
{
    // verify CRC
    const uint8_t pkt_len = 12 + data_len;
    const uint8_t crc = crc_sum_of_bytes(_buf, pkt_len - 2);
    const uint8_t expected_hi = hex2char((crc >> 4) & 0x0f);
    const uint8_t expected_lo = hex2char(crc & 0x0f);
    if (_buf[pkt_len - 2] != expected_hi || _buf[pkt_len - 1] != expected_lo) {
        return;
    }

    // ID is at bytes [7..9]
    const char *id = (const char*)&_buf[7];

    if (strncmp(id, "GIA", 3) == 0) {
        // attitude request
        send_attitude();

    } else if (strncmp(id, "VSN", 3) == 0) {
        const uint8_t data[] { '1', '.', '0', '.', '0' };
        send_packet('D', "VSN", false, data, sizeof(data));

    } else if (strncmp(id, "PA2", 3) == 0) {
        const uint8_t data[] { 'S', 'I', 'M', '_', 'T', 'P' };
        send_packet('G', "PA2", false, data, sizeof(data));

    } else if (strncmp(id, "SDC", 3) == 0) {
        // card present: any 4 bytes that are NOT all 'N'
        const uint8_t data[] { '1', '0', '0', '0' };
        send_packet('D', "SDC", false, data, sizeof(data));

    } else if (strncmp(id, "TRC", 3) == 0) {
        // tracking stopped: driver reads _msg_buff[11] as TrackingStatus;
        // [11] = data[1] = '0' = 0x30 = STOPPED_TRACKING
        const uint8_t data[] { '0', '0' };
        send_packet('D', "TRC", false, data, sizeof(data));
    }
    // all other commands (YPR, GIY, GIP, GIR, PTZ, LAT, LON, ALT, etc.) absorbed silently
}

void Topotek::send_packet(char addr2, const char id[3], bool write, const uint8_t *data, uint8_t len)
{
    const uint8_t total = 12 + len;
    if (total > PACKETLEN_MAX) {
        return;
    }

    uint8_t pkt[PACKETLEN_MAX];
    uint8_t ofs = 0;

    pkt[ofs++] = '#';
    pkt[ofs++] = 'T';
    pkt[ofs++] = 'P';
    pkt[ofs++] = 'U';
    pkt[ofs++] = (uint8_t)addr2;
    pkt[ofs++] = hex2char(len & 0x0f);   // data length as single ASCII hex char
    pkt[ofs++] = write ? 'w' : 'r';
    pkt[ofs++] = (uint8_t)id[0];
    pkt[ofs++] = (uint8_t)id[1];
    pkt[ofs++] = (uint8_t)id[2];

    for (uint8_t i = 0; i < len; i++) {
        pkt[ofs++] = data[i];
    }

    // checksum: byte sum of all preceding bytes, encoded as 2 uppercase ASCII hex chars
    const uint8_t crc = crc_sum_of_bytes(pkt, ofs);
    pkt[ofs++] = hex2char((crc >> 4) & 0x0f);
    pkt[ofs++] = hex2char(crc & 0x0f);

    write_to_autopilot((const char*)pkt, ofs);
}

void Topotek::uint16_to_hex4(uint16_t val, uint8_t buf[4])
{
    buf[0] = hex2char((val >> 12) & 0x0f);
    buf[1] = hex2char((val >>  8) & 0x0f);
    buf[2] = hex2char((val >>  4) & 0x0f);
    buf[3] = hex2char((val      ) & 0x0f);
}

#endif  // AP_SIM_TOPOTEK_ENABLED
