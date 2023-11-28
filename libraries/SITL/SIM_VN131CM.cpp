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
        strncpy((char*)&msg.buf[16], "VN131CM", 8);
        FALLTHROUGH;
    case 6:         // temperature, 2-byte sign int
        msg.buf[4] = temperature_C;
        msg.buf[5] = (temperature_C-int(temperature_C))*255;
        FALLTHROUGH;
    case 4:         // pressure, 3-byte unsigned int
        msg.buf[3] = 17;
        msg.buf[2] = 18;
        msg.buf[1] = 19;
        FALLTHROUGH;
    case 1:         // mode
        msg.buf[0] = mode;
        break;
    default:
        AP_HAL::panic("bad msg read len");
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
        FALLTHROUGH;
    case 1:
        break;
    default:
        AP_HAL::panic("Bad msg write len");
    }

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
