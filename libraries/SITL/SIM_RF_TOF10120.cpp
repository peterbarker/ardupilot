#include "SIM_RF_TOF10120.h"

#include <SITL/SITL.h>

SITL::TOF10120::TOF10120() :
    I2CRegisters_16Bit()
{
    add_register("DistanceMM", TOF10120DevReg::DISTANCE_MM, O_RDONLY);
    add_register("FilterDistanceMM", TOF10120DevReg::FILTER_DISTANCE_MM, O_RDONLY);
    add_register("DistanceDeviationMM", TOF10120DevReg::DISTANCE_DEVIATION_MM, O_RDONLY);
    add_register("Config", TOF10120DevReg::CONFIG, O_RDWR);

    reset();
}

void SITL::TOF10120::reset()
{
    set_register(TOF10120DevReg::CONFIG, (uint16_t)1);
}

void SITL::TOF10120::update(const class Aircraft &aircraft)
{
    // FIXME: fill in other registers
    set_register(TOF10120DevReg::DISTANCE_MM, htobe16(aircraft.rangefinder_range()*1000.0f));
}
