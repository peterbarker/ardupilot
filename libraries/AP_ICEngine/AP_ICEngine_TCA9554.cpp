#include "AP_ICEngine_config.h"

/*
  support for TCA9554 for starter control on I2C
 */

#if AP_ICENGINE_TCA9554_STARTER_ENABLED
#include "AP_ICEngine.h"

extern const AP_HAL::HAL& hal;

/*
  initialise TCA9554
 */
bool AP_ICEngine_TCA9554::TCA9554_init()
{
    dev_TCA9554 = std::move(hal.i2c_mgr->get_device(TCA9554_I2C_BUS, TCA9554_I2C_ADDR));
    if (!dev_TCA9554) {
        return false;
    }
    WITH_SEMAPHORE(dev_TCA9554->get_semaphore());

    dev_TCA9554->set_retries(10);

    // set outputs
    bool ret = dev_TCA9554->write_register(TCA9554_OUTPUT, TCA9554_OUT_DEFAULT);
    if (!ret) {
        return false;
    }
    ret = dev_TCA9554->write_register(TCA9554_CONF, TCA9554_PINS);
    if (!ret) {
        return false;
    }
    dev_TCA9554->set_retries(1);
    return true;
}

void AP_ICEngine_TCA9554::TCA9554_set(TCA9554_state_t value)
{
    if (value != last_state) {
        WITH_SEMAPHORE(dev_TCA9554->get_semaphore());
        // set outputs and status leds
        if (dev_TCA9554->write_register(TCA9554_OUTPUT, (~(value<<2) & 0x0C) | value)) {
            last_state = value;
        }
    }
}

void AP_ICEngine_TCA9554::set_starter(bool on)
{
    if (!initialised) {
        initialised = TCA9554_init();
        if (!initialised) {
            // waiting for power to PMU
            return;
        }
    }
    TCA9554_set(on? STARTER_ON : STARTER_OFF);
}

#endif // AP_ICENGINE_TCA9554_STARTER_ENABLED
