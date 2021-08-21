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
  Simulator for the CodevESC
*/

#include <AP_Math/AP_Math.h>

#include "SIM_CodevESC.h"
#include "SITL.h"
#include <AP_HAL/utility/sparse-endian.h>

#include "SIM_Aircraft.h"

#include <stdio.h>
#include <errno.h>

extern const AP_HAL::HAL& hal;

using namespace SITL;

// table of user settable parameters
const AP_Param::GroupInfo CodevESC::var_info[] = {

    // @Param: ENA
    // @DisplayName: Codev ESC simulator enable/disable
    // @Description: Allows you to enable (1) or disable (0) the CodevESC simulator
    // @Values: 0:Disabled,1:Enabled
    // @User: Advanced
    AP_GROUPINFO("ENA", 1, CodevESC, _enabled, 0),

    // @Param: PWOF
    // @DisplayName: Power off Codev ESC mask
    // @Description: Allows you to turn power off to the simulated ESCs.  Bits correspond to the ESC ID, *NOT* their servo channel.
    // @User: Advanced
    AP_GROUPINFO("POW", 2, CodevESC, _powered_mask, 0xfff),

    AP_GROUPEND
};

CodevESC::CodevESC() : SerialDevice::SerialDevice()
{
    AP_Param::setup_object_defaults(this, var_info);

    // initialise serial numbers and IDs
    for (uint8_t n=0; n<ARRAY_SIZE(escs); n++) {
        ESC &esc = escs[n];
        esc.ofs = n;  // so we can index for RPM, for example
        esc.id = n+1;  // really should parameterise this
    }
}

void CodevESC::update_escs()
{
    // process the power-off mask
    for (auto  &esc : escs) {
        bool should_be_on = _powered_mask & (1U<<(esc.id-1));
        switch (esc.state()) {
        case ESC::State::POWERED_OFF:
            if (should_be_on) {
                esc.set_state(ESC::State::IN_BOOTLOADER);
                esc.pwm = 0;
                esc.runs_received = 0;
            }
            break;
        case ESC::State::IN_BOOTLOADER:
        case ESC::State::POWER_UP:
        case ESC::State::RUNNING:
        case ESC::State::RUNNING_START:
            if (!should_be_on) {
                esc.set_state(ESC::State::POWERED_OFF);
                break;
            }
        }
    }

    for (auto  &esc : escs) {
        switch (esc.state()) {
        case ESC::State::POWERED_OFF:
            break;
        case ESC::State::IN_BOOTLOADER:
            // not even sure if this has a separate bootlaoder state...
            esc.set_state(ESC::State::POWER_UP);
            FALLTHROUGH;
        case ESC::State::POWER_UP:
            if (esc.runs_received < 1) {  // what should this number be?
                break;
            }
            esc.set_state(ESC::State::RUNNING_START);
            esc.running_start_ms = AP_HAL::millis();
            FALLTHROUGH;
        case ESC::State::RUNNING_START:
            if (AP_HAL::millis() - esc.running_start_ms < 10) {
                break;
            }
            esc.set_state(ESC::State::RUNNING);
            FALLTHROUGH;
        case ESC::State::RUNNING:
            // FIXME: this may not be an entirely accurate model of the
            // temperature profile of these ESCs.
            esc.temperature += esc.pwm/100000;
            esc.temperature *= 0.95;
            break;
        }
    }
}

void CodevESC::update(const class Aircraft &aircraft)
{
    if (!_enabled.get()) {
        return;
    }

    update_escs();

    update_input();
    update_send(aircraft);
}

void CodevESC::ESC::handle_message(const MessageUnion &u)
{
    // printf("%u: message received in state %u\n", id, (unsigned)state());
    switch (state()) {
    case ESC::State::POWERED_OFF:
        return;
    case ESC::State::IN_BOOTLOADER:
        return;
    case ESC::State::POWER_UP:
        return power_up_handle_message(u);
    case ESC::State::RUNNING_START:
        return;
    case ESC::State::RUNNING:
        return running_handle_message(u);
    }
    AP_HAL::panic("Unknown state");
}

void CodevESC::handle_message()
{
    const uint8_t select_pin1 = 13;
    const uint8_t select_pin2 = 14;
    const uint8_t select_pin3 = 15;
    const uint8_t selected_esc = (
        hal.gpio->read(select_pin1) << 0 |
        hal.gpio->read(select_pin2) << 1 |
        hal.gpio->read(select_pin3) << 2
        );

    if (selected_esc == 0) {
        // broadcast
        for (auto &esc : escs) {
            esc.handle_message(u);
        }
    } else {
        ESC &esc = escs[selected_esc-1];
        esc.handle_message(u);
    }
}

template <typename T>
void CodevESC::send_response(const T &r)
{
    // simcdv_debug("Sending response");
    if (write_to_autopilot((char*)&r, sizeof(r)) != sizeof(r)) {
        AP_HAL::panic("short write");
    }
}

struct RPMThing {
    uint16_t pwm : 11;
    uint16_t red : 1;
    uint16_t green : 1;
    uint16_t blue : 1;
    uint16_t feedback : 1;
    uint16_t reverse : 1;
};

void CodevESC::ESC::power_up_handle_message(const MessageUnion &u)
{
    switch ((MessageID)u.empty.msg_id) {
    case MessageID::CONFIG_BASIC: {
        config = u.config_info_basic.msg;
        break;
    }
    case MessageID::RUN: {
        const RPMThing *rpms = (RPMThing *)&u.empty.msg;
        if (ofs >= config.maxChannelInUse) {
            return;
        }
        // all zeroes might be going a bit far, but let's check:
        for (uint8_t i=0; i<config.maxChannelInUse; i++) {
            if (rpms[i].pwm != 1190) {  // magic value!
                AP_HAL::panic("%u: non-zero rpm (%u) received in powerup mode", id, rpms[i].pwm);
            }
        }
        pwm = rpms[ofs].pwm;
        runs_received++;
        return;
    }
    default:
        AP_HAL::panic("Bad message for state (%u) (state=powerup)", (unsigned)u.empty.msg_id);
    }
}

void CodevESC::ESC::running_handle_message(const MessageUnion &u)
{
    switch ((MessageID)u.empty.msg_id) {
    case MessageID::RUN: {
        if (ofs >= config.maxChannelInUse) {
            return;
        }
        const RPMThing *rpms = (RPMThing *)&u.empty.msg;
        pwm = rpms[ofs].pwm;
        return;
    }
    default:
        break;
    }
    AP_HAL::panic("Unknown message (%u)", (unsigned)u.empty.msg_id);
}

void CodevESC::consume_bytes(uint8_t count)
{
    if (count > buflen) {
        AP_HAL::panic("Consuming more bytes than in buffer?");
    }
    if (buflen == count) {
        buflen = 0;
        return;
    }
    memmove(&u.buffer[0], &u.buffer[count], buflen - count);
    buflen -= count;
}

void CodevESC::update_input()
{
    const ssize_t n = read_from_autopilot((char*)&u.buffer[buflen], ARRAY_SIZE(u.buffer) - buflen - 1);
    if (n < 0) {
        // TODO: do better here
        if (errno != EAGAIN && errno != EWOULDBLOCK && errno != 0) {
            AP_HAL::panic("Failed to read from autopilot");
        }
    } else {
        buflen += n;
    }

    if (buflen > offsetof(PackedMessage<Empty>, header) &&
        u.empty.header == CODEV_MSG_HEADER &&
        buflen > offsetof(PackedMessage<Empty>, len)+2 &&
        buflen >= u.empty.len+2) {
        const uint8_t calculated_checksum = crc8_update_seed(0, u.buffer + offsetof(PackedMessage<Empty>, len), u.empty.len+2, 0xE7);
        const uint8_t received_checksum = u.buffer[u.empty.len+3];
        if (calculated_checksum == received_checksum) {
            handle_message();
            // consume the message:
            // printf("consuming %u bytes\n", (unsigned)u.empty.len+4);
            consume_bytes(u.empty.len+4);
            return;
        } else {
            simcdv_debug("Checksum mismatch");
            abort();
        }
        return; // 1 message/loop....
    }

    // debug("Read (%d) bytes from autopilot", (signed)n);
    if (n >= 0) {
        abort();
    }
    // buflen = 0;
}

void CodevESC::update_sitl_input_pwm(struct sitl_input &input)
{
    // overwrite the SITL input values passed through from
    // sitl_model->update_model with those we're receiving serially
    for (auto &esc : escs) {
        if (esc.id > ARRAY_SIZE(input.servos)) {
            // silently ignore; input.servos is 12-long, we are
            // usually 16-long
            continue;
        }
        input.servos[esc.id-1] = esc.pwm;
    }
}

void CodevESC::send_esc_telemetry(const Aircraft &aircraft)
{
    // for (auto &esc : escs) {
    //     if (!esc.telem_request) {
    //         continue;
    //     }
    //     esc.telem_request = false;
    //     if (esc.state != ESC::State::RUNNING) {
    //         continue;
    //     }
    //     if (esc.telem_type != TLMType::ALTERNATIVE) {
    //         // no idea what "normal" looks like
    //         abort();
    //     }

    //     const int8_t temp_cdeg = esc.temperature * 100;
    //     const uint16_t voltage = aircraft.get_battery_voltage() * 100;
    //     const uint16_t current = (6 + esc.id * 100);

    //     // FIXME: the vehicle models should be supplying this RPM!
    //     const uint16_t Kv = 1000;
    //     const float p = (esc.pwm-1000)/1000.0;
    //     int16_t rpm = aircraft.get_battery_voltage() * Kv * p;

    //     const uint16_t consumption_mah = 0;
    //     const uint16_t errcount = 17;
    //     send_response(PackedMessage<ESCTelem> {
    //         esc.id,
    //         ESCTelem{temp_cdeg, voltage, current, rpm, consumption_mah, errcount}
    //     });
    // }
}

void CodevESC::update_send(const Aircraft &aircraft)
{
    send_esc_telemetry(aircraft);
}
