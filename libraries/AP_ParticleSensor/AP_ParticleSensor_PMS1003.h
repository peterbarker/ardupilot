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

http://www.aqmd.gov/docs/default-source/aq-spec/resources-page/plantower-pms1003-manual_v2-5.pdf

  DEV=/dev/serial/by-id/usb-FTDI_FT232R_USB_UART_A50285BI-if00-port0
  ./Tools/autotest/sim_vehicle.py --gdb --debug -v APMrover2 -A --uartF=uart:$DEV --speedup=1

  param set SERIAL5_PROTOCOL 30
  reboot

 */
#pragma once

#include "AP_ParticleSensor_Backend.h"
#include <AP_HAL/AP_HAL.h>

class AP_ParticleSensor_PMS1003 : public AP_ParticleSensor_Backend {
public:

    AP_ParticleSensor_PMS1003(AP_HAL::UARTDriver &port);

    virtual void update() override;

private:

    AP_HAL::UARTDriver &port;

    void yield_message();
    void handle_byte_read(uint8_t byte);

    enum class State {
        WantHeader1,
        WantHeader2,
        WantFrameLengthHigh,
        WantFrameLengthLow,
        WantData1High,
        WantData1Low,
        WantData2High,
        WantData2Low,
        WantData3High,
        WantData3Low,
        WantData4High,
        WantData4Low,
        WantData5High,
        WantData5Low,
        WantData6High,
        WantData6Low,
        WantData7High,
        WantData7Low,
        WantData8High,
        WantData8Low,
        WantData9High,
        WantData9Low,
        WantData10High,
        WantData10Low,
        WantData11High,
        WantData11Low,
        WantData12High,
        WantData12Low,
        WantData13High,
        WantData13Low,
        WantChecksum1,
        WantChecksum2,
    };
    State state = State::WantHeader1;

    uint32_t bad_chars;
    uint32_t checksum_failures;
    uint16_t framelength;

    uint16_t recv_checksum; // received from device

    class Reading
    {
    public:
        uint16_t pm1p0; //1ug/m^3
        uint16_t pm2p5; //2.5ug/m^3
        uint16_t pm10; //1ug/m^3

        uint16_t pm1p0_atmos; //1ug/m^3
        uint16_t pm2p5_atmos; //2.5ug/m^3
        uint16_t pm10_atmos; //1ug/m^3

        uint16_t pdl0p3; //particles/decilitre (>0.3um)
        uint16_t pdl0p5; //particles/decilitre (>0.5um)
        uint16_t pdl1p0; //particles/decilitre (>1.0um)
        uint16_t pdl2p5; //particles/decilitre (>2.5um)
        uint16_t pdl5p0; //particles/decilitre (>5.0um)
        uint16_t pdl10p0; //particles/decilitre (>10.0um)
    };

    Reading reading;
    uint16_t checksum; // calculated from data received
};
