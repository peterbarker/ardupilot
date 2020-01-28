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

#include "AP_ParticleSensor_PMS1003.h"

#include <stdio.h>

#include <AP_Logger/AP_Logger.h>

AP_ParticleSensor_PMS1003::AP_ParticleSensor_PMS1003(AP_HAL::UARTDriver &_port) :
        port(_port)
{
    port.begin(9600);
    port.set_flow_control(AP_HAL::UARTDriver::FLOW_CONTROL_DISABLE);
}

void AP_ParticleSensor_PMS1003::yield_message()
{
    ::fprintf(stderr, "PMS1003: %u: PM1.0=%0.1f PM2.5=%0.1f PM10=%0.1f (bad=%u cksum fails=%u)\n", AP_HAL::millis(), reading.pm1p0/10.0, reading.pm2p5/10.0, reading.pm10/10.0, bad_chars, checksum_failures);
    const uint64_t now = AP_HAL::micros64();
    AP::logger().Write(
        "P103",
        "TimeUS,pm1p0,pm2p5,pm10",
        "Qhhh",
        now, reading.pm1p0, reading.pm2p5, reading.pm1p0);
}

void AP_ParticleSensor_PMS1003::handle_byte_read(const uint8_t byte)
{
    fprintf(stderr, "Got byte (0x%02x) (state=%u)\n", byte, (uint8_t)state);
    switch(state) {
    case State::WantHeader1:
        checksum = byte;
        switch(byte) {
        case 0x42:
            state = State::WantHeader2;
            break;
        default:
            bad_chars++;
            break;
        }
        break;
    case State::WantHeader2:
        checksum += byte;
        switch(byte) {
        case 0x4d:
            state = State::WantFrameLengthHigh;
            framelength = 0;
            break;
        default:
            bad_chars++;
            state = State::WantHeader1;
            break;
        }
        break;
    case State::WantFrameLengthHigh:
        checksum += byte;
        framelength = byte << 8;
        state = State::WantFrameLengthLow;
        break;
    case State::WantFrameLengthLow:
        checksum += byte;
        framelength |= byte;
        // ::fprintf(stderr, "framelenght=%u\n", framelength);
        state = State::WantData1High;
        break;

    case State::WantData1High:
        reading.pm1p0 = byte << 8;
        state = State::WantData1Low;
        checksum += byte;
        break;
    case State::WantData1Low:
        reading.pm1p0 |= byte;
        state = State::WantData2High;
        checksum += byte;
        break;

    case State::WantData2High:
        reading.pm2p5 = byte << 8;
        state = State::WantData2Low;
        checksum += byte;
        break;
    case State::WantData2Low:
        reading.pm2p5 |= byte;
        state = State::WantData3High;
        checksum += byte;
        break;

    case State::WantData3High:
        reading.pm10 = byte << 8;
        state = State::WantData3Low;
        checksum += byte;
        break;
    case State::WantData3Low:
        reading.pm10 |= byte;
        state = State::WantData4High;
        checksum += byte;
        break;

    case State::WantData4High:
        reading.pm1p0_atmos = byte << 8;
        state = State::WantData4Low;
        checksum += byte;
        break;
    case State::WantData4Low:
        reading.pm1p0_atmos |= byte;
        state = State::WantData5High;
        checksum += byte;
        break;

    case State::WantData5High:
        reading.pm2p5_atmos = byte << 8;
        state = State::WantData5Low;
        checksum += byte;
        break;
    case State::WantData5Low:
        reading.pm2p5_atmos |= byte;
        state = State::WantData6High;
        checksum += byte;
        break;

    case State::WantData6High:
        reading.pm10_atmos = byte << 8;
        state = State::WantData6Low;
        checksum += byte;
        break;
    case State::WantData6Low:
        reading.pm10_atmos |= byte;
        state = State::WantData7High;
        checksum += byte;
        break;

    case State::WantData7High:
        reading.pdl0p3 = byte << 8;
        state = State::WantData7Low;
        checksum += byte;
        break;
    case State::WantData7Low:
        reading.pdl0p3 |= byte;
        state = State::WantData8High;
        checksum += byte;
        break;

    case State::WantData8High:
        reading.pdl0p5 = byte << 8;
        state = State::WantData8Low;
        checksum += byte;
        break;
    case State::WantData8Low:
        reading.pdl0p5 |= byte;
        state = State::WantData9High;
        checksum += byte;
        break;

    case State::WantData9High:
        reading.pdl1p0 = byte << 8;
        state = State::WantData9Low;
        checksum += byte;
        break;
    case State::WantData9Low:
        reading.pdl1p0 |= byte;
        if (framelength == 20) {
            // data sheet I have gives more fields than my device does....
            state = State::WantChecksum1;
        } else {
            state = State::WantData10High;
        }
        checksum += byte;
        break;

    case State::WantData10High:
        reading.pdl2p5 = byte << 8;
        state = State::WantData10Low;
        checksum += byte;
        break;
    case State::WantData10Low:
        reading.pdl2p5 |= byte;
        state = State::WantData11High;
        checksum += byte;
        break;

    case State::WantData11High:
        reading.pdl5p0 = byte << 8;
        state = State::WantData11Low;
        checksum += byte;
        break;
    case State::WantData11Low:
        reading.pdl5p0 |= byte;
        state = State::WantData12High;
        checksum += byte;
        break;

    case State::WantData12High:
        reading.pdl10p0 = byte << 8;
        state = State::WantData12Low;
        checksum += byte;
        break;
    case State::WantData12Low:
        reading.pdl10p0 |= byte;
        state = State::WantData13High;
        checksum += byte;
        break;

    case State::WantData13High:
        //RESERVED
        state = State::WantData13Low;
        checksum += byte;
        break;
    case State::WantData13Low:
        //RESERVED
        state = State::WantChecksum1;
        checksum += byte;
        break;

    case State::WantChecksum1:
        recv_checksum = byte << 8;
        state = State::WantChecksum2;
        break;
    case State::WantChecksum2:
        recv_checksum |= byte;
        if (recv_checksum != checksum) {
            checksum_failures++;
            fprintf(stderr, "checksum failure (got=%u) vs (calced=%u)\n", recv_checksum, checksum);
            state = State::WantHeader1;
            break;
        }
        state = State::WantHeader1;
        yield_message();
        break;
    }
}

void AP_ParticleSensor_PMS1003::update()
{
    uint8_t count = 10; // maximum characters to process per update
    while (count-- > 0 && port.available()) {
        const uint8_t next = port.read();
        handle_byte_read((uint8_t)next);
    }
}
