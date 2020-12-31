#include "SIM_RF_GP2Y0E03.h"

#include <stdio.h>

void SITL::GP2Y0E03::software_reset()
{
    set_register(GP2Y0E03DevReg::SOFTWARE_RESET, (uint8_t)0);
    set_register(GP2Y0E03DevReg::PULSE_WIDTH, (uint8_t)0x07);  // 320ms
    set_register(GP2Y0E03DevReg::SHIFT_BIT, (uint8_t)0x01);  // 128cm
}

void SITL::GP2Y0E03::update(const class Aircraft &aircraft)
{
    const uint32_t now = AP_HAL::millis();
    if (now - last_update_ms < 10) {  // 100Hz update
        return;
    }
    last_update_ms = now;

    if (get_register(GP2Y0E03DevReg::SOFTWARE_RESET) == SOFTWARE_RESET_MAGIC_VALUE) {
        gcs().send_text(MAV_SEVERITY_INFO, "SIM_GP2Y0E03 Reset");
        software_reset();
    }

    const float rangefinder_range = aircraft.rangefinder_range();
    const uint32_t rangefinder_range_cm = rangefinder_range * 100;


//    rangefinder_range = ((distance1*16)+distance2)/16/(2<<shift);

    set_register(GP2Y0E03DevReg::DISTANCE1,
                 uint8_t(rangefinder_range_cm >> (get_register(GP2Y0E03DevReg::SHIFT_BIT))));
    set_register(GP2Y0E03DevReg::DISTANCE2, (uint8_t)0);

}
