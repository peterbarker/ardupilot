#pragma once

#include "SIM_config.h"

#ifdef AP_SIM_FSO_POWERSTACK_ENABLED

#ifndef AP_SIM_POWERSTACK_SFML_ENABLED
#define AP_SIM_POWERSTACK_SFML_ENABLED 0
#endif

#if AP_SIM_POWERSTACK_SFML_ENABLED
#include <stdint.h>
#include <pthread.h>
#endif

namespace SITL {

class FSO_PowerStack
{
public:

    void init();
    void update(class Aircraft &aircraft);

    void set_dac(class TI_DACx3204 &_dac) { dac = &_dac; }
    void set_ina(class INA2xx *_ina, uint8_t _ina_count) {
        ina = _ina;
        ina_count = _ina_count;
    }

private:

    static constexpr uint8_t height = 200;
    static constexpr uint8_t width = height;

#if AP_SIM_POWERSTACK_SFML_ENABLED
    pthread_t thread;
    static void *update_thread_start(void *obj);
    void update_thread(void);
#endif

    uint32_t last_fan_toggle_us;

    TI_DACx3204 *dac;

    INA2xx *ina;
    uint8_t ina_count;
};

};  // close namespace SITL

#endif  // AP_SIM_FSO_POWERSTACK_ENABLED
