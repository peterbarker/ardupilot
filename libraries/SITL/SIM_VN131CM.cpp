#include "SIM_config.h"

#if AP_SIM_VN131CM_ENABLED

#include "SIM_VN131CM.h"

using namespace SITL;

int VN131CM::rd(I2C::i2c_rdwr_ioctl_data *&data)
{
    if (data->nmsgs != 1) {
        AP_HAL::panic("Bad number of messages");
    }

    auto &msg { data->msgs[0] };

    float temperature_C;
    float pressure_Pa;
    get_pressure_temperature_readings(pressure_Pa, temperature_C);
    const uint32_t pressure = pressure_field_value(pressure_Pa);

    switch (msg.len) {
    case 24:        // build number, 6 bytes ascii, null-terminated
        strncpy((char*)&msg.buf[18], "ABCDE", 6);
        FALLTHROUGH;
    case 18:        // serial number, 4 bytes hex
        msg.buf[14] = 0xDE;
        msg.buf[15] = 0xAD;
        msg.buf[16] = 0xBE;
        msg.buf[17] = 0xE5;
        FALLTHROUGH;
    case 14:         // model, right-reading ascii with null-termination
        strncpy((char*)&msg.buf[6], "VN131CM", 8);
        FALLTHROUGH;
    case 6:         // temperature, 2-byte sign int
        msg.buf[5] = (abs(temperature_C) - int(abs(temperature_C))) * 256;
        msg.buf[4] = temperature_C;
        FALLTHROUGH;
    case 4:         // pressure, 3-byte unsigned int
        msg.buf[3] = pressure;
        msg.buf[2] = pressure >> 8;
        msg.buf[1] = pressure >> 16;
        FALLTHROUGH;
    case 1:         // mode
        msg.buf[0] = reg_mode;
        break;
    default:
        AP_HAL::panic("bad msg read len");
    }

    if (one_cycle_latency) {
        reg_mode = new_reg_mode;
        reg_rate_control = new_reg_rate_control;
        one_cycle_latency = false;
    }

    return 0;
}

int VN131CM::wr(I2C::i2c_rdwr_ioctl_data *&data)
{
    if (data->nmsgs != 1) {
        AP_HAL::panic("Bad number of messages");
    }

    auto &msg { data->msgs[0] };

    switch (msg.len) {
    case 2:
        // setting both control and rate registers
        new_reg_rate_control = msg.buf[1];
        FALLTHROUGH;
    case 1:
        new_reg_mode = msg.buf[0];
        break;
    default:
        AP_HAL::panic("Bad msg write len");
    }

    one_cycle_latency = true;

    return 0;
}

int VN131CM::rdwr(I2C::i2c_rdwr_ioctl_data *&data)
{
    if (data->msgs[0].flags == I2C_M_RD) {
        return rd(data);
    }

    return wr(data);
}

void VN131CM::update(const class Aircraft &aircraft)
{
}

// returns the three-bytes required for a pressure reading:
uint32_t VN131CM::pressure_field_value(float press) const
{
    const float vn131cm_range[8] = {23.6, 27.5, 31.5, 35.4, 39.4, 43.3, 47.2, 51.2}; //all in inH2O
    const float _current_range_val = vn131cm_range[reg_mode & 0x7];

    const float inH20_to_Pa = 249.08f;

    float diff_press_inH2O = press / inH20_to_Pa;

    // const float margin = 29491.2f;

    const uint32_t dp_raw = ((diff_press_inH2O * 15099493.5f) / _current_range_val) + 8388607.5f;
    return dp_raw;
}

// FIXME: this was copied in from MS5525
void VN131CM::get_pressure_temperature_readings(float &P_Pa, float &Temp_C)
{
    float sim_alt = AP::sitl()->state.altitude;
    sim_alt += 2 * rand_float();

    float sigma, delta, theta;
    AP_Baro::SimpleAtmosphere(sim_alt * 0.001f, sigma, delta, theta);

    // To Do: Add a sensor board temperature offset parameter
    Temp_C = (KELVIN_TO_C(SSL_AIR_TEMPERATURE * theta)) + 25.0;
    const uint8_t instance = 0;  // TODO: work out which sensor this is
    P_Pa = AP::sitl()->state.airspeed_raw_pressure[instance] + AP::sitl()->airspeed[instance].offset;
}

#endif  // AP_SIM_VN131CM_ENABLED
