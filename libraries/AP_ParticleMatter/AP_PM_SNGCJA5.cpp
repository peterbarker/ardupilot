#include <AP_HAL/AP_HAL.h>
#include "AP_PM_SNGCJA5.h"
#include <stdio.h>
#include <utility>
#include <AP_HAL/I2CDevice.h>
#include <GCS_MAVLink/GCS.h>

extern const AP_HAL::HAL& hal;

#define SNGCJA5_I2C_ADDRESS 0x33

#define SNGCJA5_STATUS 0x26

#define SNGCJA5_PM1_0_LL 0x00
#define SNGCJA5_PM1_0_LH 0x01
#define SNGCJA5_PM1_0_HL 0x02
#define SNGCJA5_PM1_0_HH 0x03

#define SNGCJA5_PM2_5_LL 0x04
#define SNGCJA5_PM2_5_LH 0x05
#define SNGCJA5_PM2_5_HL 0x06
#define SNGCJA5_PM2_5_HH 0x07

#define SNGCJA5_PM10_0_LL 0x08
#define SNGCJA5_PM10_0_LH 0x09
#define SNGCJA5_PM10_0_HL 0x0A
#define SNGCJA5_PM10_0_HH 0x0B

#define SNGCJA5_PC_0_5_L 0x0C
#define SNGCJA5_PC_0_5_H 0x0D

#define SNGCJA5_PC_1_0_L 0x0E
#define SNGCJA5_PC_1_0_H 0x0F

#define SNGCJA5_PC_2_5_L 0x10
#define SNGCJA5_PC_2_5_H 0x11

#define SNGCJA5_PC_5_0_L 0x14
#define SNGCJA5_PC_5_0_H 0x15

#define SNGCJA5_PC_7_5_L 0x16
#define SNGCJA5_PC_7_5_H 0x17

#define SNGCJA5_PC_10_0_L 0x18
#define SNGCJA5_PC_10_0_H 0x19

void AP_PM_SNGCJA5::init()
{
    FOREACH_I2C(i) {
        if (init(i)) {
            return;
        }
    }
    gcs().send_text(MAV_SEVERITY_INFO, "No SNGCJA5 found");
}

bool AP_PM_SNGCJA5::init(int8_t bus)
{
    dev = std::move(hal.i2c_mgr->get_device(bus, SNGCJA5_I2C_ADDRESS));
    if (!dev) {
        return false;
    }

    // read at 2Hz
    printf("Starting Particle Matter Sensor on I2C\n");

    dev->register_periodic_callback(500000, FUNCTOR_BIND_MEMBER(&AP_PM_SNGCJA5::read_frames, void));
    return true;
}

void AP_PM_SNGCJA5::read_frames(void)
{
    uint8_t val[1];
    if (!dev->read_registers(SNGCJA5_STATUS, val, sizeof(val))) {
        return;
    }

    gcs().send_text(MAV_SEVERITY_INFO,"Particle Sensor: %u", (unsigned)val[0]);
}

// periodically called from vehicle code
void AP_PM_SNGCJA5::update()
{
    read_frames();
}
