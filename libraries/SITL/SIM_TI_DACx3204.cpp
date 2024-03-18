#include "SIM_config.h"

#if AP_SIM_DAC_TI_DACx3204_ENABLED

#include "SIM_TI_DACx3204.h"

using namespace SITL;

#include <GCS_MAVLink/GCS.h>

TI_DACx3204::TI_DACx3204() :
    I2CDevice(),
    I2CRegisters_16Bit()
{

    add_register("NOP", TI_DACx3204RegEnum::NOP, SITL::I2CRegisters::RegMode::WRONLY);  // RO as we should never be touching it

    const auto MH_MODE = SITL::I2CRegisters::RegMode::WRONLY;
    const auto ML_MODE = SITL::I2CRegisters::RegMode::WRONLY;
    const auto VOUT_MODE = SITL::I2CRegisters::RegMode::WRONLY;

    // FIXME: iterate!
    add_register("DAC-0-MARGIN-HIGH", TI_DACx3204RegEnum::DAC_0_MARGIN_HIGH, MH_MODE);
    add_register("DAC-0-MARGIN-LOW", TI_DACx3204RegEnum::DAC_0_MARGIN_LOW, ML_MODE);
    add_register("DAC-0-VOUT-CMP-CONFIG", TI_DACx3204RegEnum::DAC_0_VOUT_CMP_CONFIG, VOUT_MODE);
    add_register("DAC-0-VOUT-MISC-CONFIG", TI_DACx3204RegEnum::DAC_0_IOUT_MISC_CONFIG, SITL::I2CRegisters::RegMode::RDONLY);
    add_register("DAC-0-VOUT-MODE-CONFIG", TI_DACx3204RegEnum::DAC_0_CMP_MODE_CONFIG, SITL::I2CRegisters::RegMode::RDONLY);
    add_register("DAC-0-FUNC-CONFIG", TI_DACx3204RegEnum::DAC_0_FUNC_CONFIG, SITL::I2CRegisters::RegMode::RDONLY);

    add_register("DAC-1-MARGIN-HIGH", TI_DACx3204RegEnum::DAC_1_MARGIN_HIGH, MH_MODE);
    add_register("DAC-1-MARGIN-LOW", TI_DACx3204RegEnum::DAC_1_MARGIN_LOW, ML_MODE);
    add_register("DAC-1-VOUT-CMP-CONFIG", TI_DACx3204RegEnum::DAC_1_VOUT_CMP_CONFIG, VOUT_MODE);
    add_register("DAC-1-VOUT-MISC-CONFIG", TI_DACx3204RegEnum::DAC_1_IOUT_MISC_CONFIG, SITL::I2CRegisters::RegMode::RDONLY);
    add_register("DAC-1-VOUT-MODE-CONFIG", TI_DACx3204RegEnum::DAC_1_CMP_MODE_CONFIG, SITL::I2CRegisters::RegMode::RDONLY);
    add_register("DAC-1-FUNC-CONFIG", TI_DACx3204RegEnum::DAC_1_FUNC_CONFIG, SITL::I2CRegisters::RegMode::RDONLY);

    add_register("DAC-2-MARGIN-HIGH", TI_DACx3204RegEnum::DAC_2_MARGIN_HIGH, MH_MODE);
    add_register("DAC-2-MARGIN-LOW", TI_DACx3204RegEnum::DAC_2_MARGIN_LOW, ML_MODE);
    add_register("DAC-2-VOUT-CMP-CONFIG", TI_DACx3204RegEnum::DAC_2_VOUT_CMP_CONFIG, VOUT_MODE);
    add_register("DAC-2-VOUT-MISC-CONFIG", TI_DACx3204RegEnum::DAC_2_IOUT_MISC_CONFIG, SITL::I2CRegisters::RegMode::RDONLY);
    add_register("DAC-2-VOUT-MODE-CONFIG", TI_DACx3204RegEnum::DAC_2_CMP_MODE_CONFIG, SITL::I2CRegisters::RegMode::RDONLY);
    add_register("DAC-2-FUNC-CONFIG", TI_DACx3204RegEnum::DAC_2_FUNC_CONFIG, SITL::I2CRegisters::RegMode::RDONLY);

    add_register("DAC-3-MARGIN-HIGH", TI_DACx3204RegEnum::DAC_3_MARGIN_HIGH, MH_MODE);
    add_register("DAC-3-MARGIN-LOW", TI_DACx3204RegEnum::DAC_3_MARGIN_LOW, ML_MODE);
    add_register("DAC-3-VOUT-CMP-CONFIG", TI_DACx3204RegEnum::DAC_3_VOUT_CMP_CONFIG, VOUT_MODE);
    add_register("DAC-3-VOUT-MISC-CONFIG", TI_DACx3204RegEnum::DAC_3_IOUT_MISC_CONFIG, SITL::I2CRegisters::RegMode::RDONLY);
    add_register("DAC-3-VOUT-MODE-CONFIG", TI_DACx3204RegEnum::DAC_3_CMP_MODE_CONFIG, SITL::I2CRegisters::RegMode::RDONLY);
    add_register("DAC-3-FUNC-CONFIG", TI_DACx3204RegEnum::DAC_3_FUNC_CONFIG, SITL::I2CRegisters::RegMode::RDONLY);

    const auto DATA_MODE = SITL::I2CRegisters::RegMode::WRONLY;
    add_register("DAC-0-DATA", TI_DACx3204RegEnum::DAC_0_DATA, DATA_MODE);
    add_register("DAC-1-DATA", TI_DACx3204RegEnum::DAC_1_DATA, DATA_MODE);
    add_register("DAC-2-DATA", TI_DACx3204RegEnum::DAC_2_DATA, DATA_MODE);
    add_register("DAC-3-DATA", TI_DACx3204RegEnum::DAC_3_DATA, DATA_MODE);

    add_register("COMMON-CONFIG", TI_DACx3204RegEnum::COMMON_CONFIG, SITL::I2CRegisters::RegMode::RDWR);
    add_register("COMMON-TRIGGER", TI_DACx3204RegEnum::COMMON_TRIGGER, SITL::I2CRegisters::RegMode::RDONLY);
    add_register("COMMON-DAC-TRIGGER", TI_DACx3204RegEnum::COMMON_DAC_TRIGGER, SITL::I2CRegisters::RegMode::RDONLY);

    // create objects for each of the channels to handle the dynamics
    // of updating each.  The channel gets references to the relevant
    // configuration words.
    dacs[0] = new DAC{
        word[(uint8_t)TI_DACx3204RegEnum::DAC_0_MARGIN_HIGH],
        word[(uint8_t)TI_DACx3204RegEnum::DAC_0_MARGIN_LOW],
        word[(uint8_t)TI_DACx3204RegEnum::DAC_0_VOUT_CMP_CONFIG],
        word[(uint8_t)TI_DACx3204RegEnum::DAC_0_IOUT_MISC_CONFIG],
        word[(uint8_t)TI_DACx3204RegEnum::DAC_0_CMP_MODE_CONFIG],
        word[(uint8_t)TI_DACx3204RegEnum::DAC_0_FUNC_CONFIG],
        word[(uint8_t)TI_DACx3204RegEnum::DAC_0_DATA]
    };
    dacs[1] = new DAC{
        word[(uint8_t)TI_DACx3204RegEnum::DAC_1_MARGIN_HIGH],
        word[(uint8_t)TI_DACx3204RegEnum::DAC_1_MARGIN_LOW],
        word[(uint8_t)TI_DACx3204RegEnum::DAC_1_VOUT_CMP_CONFIG],
        word[(uint8_t)TI_DACx3204RegEnum::DAC_1_IOUT_MISC_CONFIG],
        word[(uint8_t)TI_DACx3204RegEnum::DAC_1_CMP_MODE_CONFIG],
        word[(uint8_t)TI_DACx3204RegEnum::DAC_1_FUNC_CONFIG],
        word[(uint8_t)TI_DACx3204RegEnum::DAC_1_DATA]
    };
    dacs[2] = new DAC{
        word[(uint8_t)TI_DACx3204RegEnum::DAC_2_MARGIN_HIGH],
        word[(uint8_t)TI_DACx3204RegEnum::DAC_2_MARGIN_LOW],
        word[(uint8_t)TI_DACx3204RegEnum::DAC_2_VOUT_CMP_CONFIG],
        word[(uint8_t)TI_DACx3204RegEnum::DAC_2_IOUT_MISC_CONFIG],
        word[(uint8_t)TI_DACx3204RegEnum::DAC_2_CMP_MODE_CONFIG],
        word[(uint8_t)TI_DACx3204RegEnum::DAC_2_FUNC_CONFIG],
        word[(uint8_t)TI_DACx3204RegEnum::DAC_2_DATA],
    };
    dacs[3] = new DAC{
        word[(uint8_t)TI_DACx3204RegEnum::DAC_3_MARGIN_HIGH],
        word[(uint8_t)TI_DACx3204RegEnum::DAC_3_MARGIN_LOW],
        word[(uint8_t)TI_DACx3204RegEnum::DAC_3_VOUT_CMP_CONFIG],
        word[(uint8_t)TI_DACx3204RegEnum::DAC_3_IOUT_MISC_CONFIG],
        word[(uint8_t)TI_DACx3204RegEnum::DAC_3_CMP_MODE_CONFIG],
        word[(uint8_t)TI_DACx3204RegEnum::DAC_3_FUNC_CONFIG],
        word[(uint8_t)TI_DACx3204RegEnum::DAC_3_DATA],
    };
}

void TI_DACx3204::reset()
{
    set_register(TI_DACx3204RegEnum::NOP, 0);
    set_register(TI_DACx3204RegEnum::DAC_0_MARGIN_HIGH, 0);
    set_register(TI_DACx3204RegEnum::DAC_0_MARGIN_LOW, 0);
    set_register(TI_DACx3204RegEnum::DAC_0_VOUT_CMP_CONFIG, 0);
    set_register(TI_DACx3204RegEnum::DAC_0_IOUT_MISC_CONFIG, 0);
    set_register(TI_DACx3204RegEnum::DAC_0_CMP_MODE_CONFIG, 0);
    set_register(TI_DACx3204RegEnum::DAC_0_FUNC_CONFIG, 0);

    set_register(TI_DACx3204RegEnum::DAC_1_MARGIN_HIGH, 0);
    set_register(TI_DACx3204RegEnum::DAC_1_MARGIN_LOW, 0);
    set_register(TI_DACx3204RegEnum::DAC_1_VOUT_CMP_CONFIG, 0);
    set_register(TI_DACx3204RegEnum::DAC_1_IOUT_MISC_CONFIG, 0);
    set_register(TI_DACx3204RegEnum::DAC_1_CMP_MODE_CONFIG, 0);
    set_register(TI_DACx3204RegEnum::DAC_1_FUNC_CONFIG, 0);

    set_register(TI_DACx3204RegEnum::DAC_2_MARGIN_HIGH, 0);
    set_register(TI_DACx3204RegEnum::DAC_2_MARGIN_LOW, 0);
    set_register(TI_DACx3204RegEnum::DAC_2_VOUT_CMP_CONFIG, 0);
    set_register(TI_DACx3204RegEnum::DAC_2_IOUT_MISC_CONFIG, 0);
    set_register(TI_DACx3204RegEnum::DAC_2_CMP_MODE_CONFIG, 0);
    set_register(TI_DACx3204RegEnum::DAC_2_FUNC_CONFIG, 0);

    set_register(TI_DACx3204RegEnum::DAC_3_MARGIN_HIGH, 0);
    set_register(TI_DACx3204RegEnum::DAC_3_MARGIN_LOW, 0);
    set_register(TI_DACx3204RegEnum::DAC_3_VOUT_CMP_CONFIG, 0);
    set_register(TI_DACx3204RegEnum::DAC_3_IOUT_MISC_CONFIG, 0);
    set_register(TI_DACx3204RegEnum::DAC_3_CMP_MODE_CONFIG, 0);
    set_register(TI_DACx3204RegEnum::DAC_3_FUNC_CONFIG, 0);

    set_register(TI_DACx3204RegEnum::DAC_0_DATA, 0);
    set_register(TI_DACx3204RegEnum::DAC_1_DATA, 0);
    set_register(TI_DACx3204RegEnum::DAC_2_DATA, 0);
    set_register(TI_DACx3204RegEnum::DAC_3_DATA, 0);

    set_register(TI_DACx3204RegEnum::COMMON_CONFIG, 0xFFF);
    set_register(TI_DACx3204RegEnum::COMMON_TRIGGER, 0);
    set_register(TI_DACx3204RegEnum::COMMON_DAC_TRIGGER, 0);

    for (auto &dac : dacs) {
        dac->reset();
    }
}

// assert that data is either not a register write, or that it's OK
// for that register write to happen
void TI_DACx3204::assert_register_write_ok(I2C::i2c_rdwr_ioctl_data *&data)
{
    if (data->nmsgs != 1) {
        // not a register write
        return;
    }

    const uint8_t reg_base_addr = data->msgs[0].buf[0];

    // check that we don't command voltage output before we configure
    // what that voltage should be:
    switch (reg_base_addr) {
    case TI_DACx3204RegEnum::DAC_0_DATA:
        dacs[0]->data_reg_written = true;
        break;
    case TI_DACx3204RegEnum::DAC_1_DATA:
        dacs[1]->data_reg_written = true;
        break;
    case TI_DACx3204RegEnum::DAC_2_DATA:
        dacs[2]->data_reg_written = true;
        break;
    case TI_DACx3204RegEnum::DAC_3_DATA:
        dacs[3]->data_reg_written = true;
        break;
    default:
        break;
    }

    switch (reg_base_addr) {
    case TI_DACx3204RegEnum::DAC_0_MARGIN_HIGH:
    case TI_DACx3204RegEnum::DAC_0_MARGIN_LOW:
    case TI_DACx3204RegEnum::DAC_0_VOUT_CMP_CONFIG:
    case TI_DACx3204RegEnum::DAC_0_IOUT_MISC_CONFIG:
    case TI_DACx3204RegEnum::DAC_0_CMP_MODE_CONFIG:
    case TI_DACx3204RegEnum::DAC_0_FUNC_CONFIG:
        if (!dacs[0]->data_reg_written) {
            AP_HAL::panic("Configure before data set");
        }
        break;

    case TI_DACx3204RegEnum::DAC_1_MARGIN_HIGH:
    case TI_DACx3204RegEnum::DAC_1_MARGIN_LOW:
    case TI_DACx3204RegEnum::DAC_1_VOUT_CMP_CONFIG:
    case TI_DACx3204RegEnum::DAC_1_IOUT_MISC_CONFIG:
    case TI_DACx3204RegEnum::DAC_1_CMP_MODE_CONFIG:
    case TI_DACx3204RegEnum::DAC_1_FUNC_CONFIG:
        if (!dacs[1]->data_reg_written) {
            AP_HAL::panic("Configure before data set");
        }
        break;

    case TI_DACx3204RegEnum::DAC_2_MARGIN_HIGH:
    case TI_DACx3204RegEnum::DAC_2_MARGIN_LOW:
    case TI_DACx3204RegEnum::DAC_2_VOUT_CMP_CONFIG:
    case TI_DACx3204RegEnum::DAC_2_IOUT_MISC_CONFIG:
    case TI_DACx3204RegEnum::DAC_2_CMP_MODE_CONFIG:
    case TI_DACx3204RegEnum::DAC_2_FUNC_CONFIG:
        if (!dacs[2]->data_reg_written) {
            AP_HAL::panic("Configure before data set");
        }
        break;

    case TI_DACx3204RegEnum::DAC_3_MARGIN_HIGH:
    case TI_DACx3204RegEnum::DAC_3_MARGIN_LOW:
    case TI_DACx3204RegEnum::DAC_3_VOUT_CMP_CONFIG:
    case TI_DACx3204RegEnum::DAC_3_IOUT_MISC_CONFIG:
    case TI_DACx3204RegEnum::DAC_3_CMP_MODE_CONFIG:
    case TI_DACx3204RegEnum::DAC_3_FUNC_CONFIG:
        if (!dacs[3]->data_reg_written) {
            AP_HAL::panic("Configure before data set");
        }
        break;
    }

    if (registers_are_locked) {
        if (reg_base_addr != TI_DACx3204RegEnum::COMMON_TRIGGER) {
            AP_HAL::panic("write to registers while they are locked");
        }
        const uint16_t register_value = data->msgs[0].buf[2] << 8 | data->msgs[0].buf[1];
        if (((register_value & common_trigger_reset_bits_mask) >> 8) != common_trigger_reset_bits_reset_value) {
            AP_HAL::panic("bad reset register value");
        }
    }
}

int TI_DACx3204::rdwr(I2C::i2c_rdwr_ioctl_data *&data)
{
    assert_register_write_ok(data);
    return I2CRegisters_16Bit::rdwr(data);
}

// called periodically by simulation code; should read registers
// written by autopilot and act on them, updating state based on those
// registers and time passing.
void TI_DACx3204::update(const class Aircraft &aircraft)
{
    // handle changed in COMMON-TRIGGER
    const uint16_t common_trigger = get_reg_value(TI_DACx3204RegEnum::COMMON_TRIGGER);
    // reset triggered by 0b1010 in common-trigger; see page 61
    const uint16_t reset_bits = (common_trigger & common_trigger_reset_bits_mask) >> 8;
    if (reset_bits == common_trigger_reset_bits_reset_value) {
        reset();
    } else if (reset_bits) {
        AP_HAL::panic("Unexpected reset bits value");
    }

    // handle changes in COMMON-CONFIG (page 60)
    const uint16_t common_config = get_reg_value(TI_DACx3204RegEnum::COMMON_CONFIG);
    const bool lock_bit_set = (common_config & 1U<<14);  // 14 is DEV-LOCK
    if (lock_bit_set && !registers_are_locked) {
        registers_are_locked = true;
    } else if (!lock_bit_set && registers_are_locked) {
        if (((common_trigger >> 12) & 0b111) != 0b101) {  // DEV-UNLOCK magic value
            AP_HAL::panic("Attempt to unlock registers with bad DEV-UNLOCK value");
        }
    }

    for (auto &dac : dacs) {
        dac->update();
    }


    static uint32_t last_output_ms;
    const uint32_t now_ms = AP_HAL::millis();
    if (now_ms - last_output_ms > 5000) {
        last_output_ms = now_ms;
        for (uint8_t i=0; i<ARRAY_SIZE(dacs); i++) {
            const auto *dac = dacs[i];
            if (dac == nullptr) {
                continue;
            }
            GCS_SEND_TEXT(MAV_SEVERITY_INFO, "DAC %u output: v=%f", i, dac->get_voltage());
        }
    }
}

// actual DAC constructor:
TI_DACx3204::DAC::DAC(uint16_t &_margin_high,
                      uint16_t &_margin_low,
                      uint16_t &_vout_cmp_config,
                      uint16_t &_vout_misc_config,
                      uint16_t &_vout_mode_config,
                      uint16_t &_vout_func_config,
                      uint16_t &_data) :
    margin_high{_margin_high},
    margin_low{_margin_low},
    vout_cmp_config{_vout_cmp_config},
    vout_misc_config{_vout_misc_config},
    vout_mode_config{_vout_mode_config},
    vout_func_config{_vout_func_config},
    data{_data}
{
}

// power-on-reset:
void TI_DACx3204::DAC::reset()
{
}

void TI_DACx3204::DAC::update()
{
    // check configuration here

    // adjust output voltage
    // const auto *sitl = AP::sitl();

    output = (be16toh(data)>>4) * 3.3 / 4095.0; // FIXME constant;
}

#endif  // AP_SIM_DAC_TI_DACx3204_ENABLED
