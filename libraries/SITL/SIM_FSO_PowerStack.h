#pragma once

/*
# configure and build:
./waf configure --debug --board=sitl_periph_fso_powerstack --enable-powerstack-sfml --enable-sfml
./waf AP_Periph

# run it
gdb --args ./build/sitl_periph_fso_powerstack/bin/AP_Periph --defaults libraries/AP_HAL_ChibiOS/hwdef/FSOPowerStack/defaults.parm

# simulation will pause until a normal vehicle simulation starts

./Tools/autotest/sim_vehicle.py -v ArduCopter --gdb --debug --mcast
param set CAN_P1_DRIVER 1
reboot
*/

#include "SIM_config.h"

/*
 * TODO:
 *  - fans
 *  - enable/disable pins
 *  - simulated buttons
 */

#ifdef AP_SIM_FSO_POWERSTACK_ENABLED

#ifndef AP_SIM_POWERSTACK_SFML_ENABLED
#define AP_SIM_POWERSTACK_SFML_ENABLED 0
#endif

#if AP_SIM_POWERSTACK_SFML_ENABLED
#include <stdint.h>
#include <pthread.h>
#include <SFML/Graphics/CircleShape.hpp>
#include <SFML/Window/Event.hpp>
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
#endif  // AP_SIM_POWERSTACK_SFML_ENABLED

    uint32_t last_fan_toggle_us;

    TI_DACx3204 *dac;

    INA2xx *ina;
    uint8_t ina_count;

#if AP_SIM_POWERSTACK_SFML_ENABLED
    static struct Button {
        Button(const char *_label, float _posx, float _posy, float _diameter) :
            label{_label},
            posx{_posx},
            posy{_posy},
            diameter{_diameter}
            { }

        const char *label;
        float posx;
        float posy;
        float diameter;
        float getRadius() const { return diameter/2; }
        sf::CircleShape shape{10};
    } buttons[];

    void handle_MouseButtonPressed(const sf::Event &event);
    void handle_MouseButtonReleased(const sf::Event &event);
    const FSO_PowerStack::Button *find_button_for_MouseButton(const sf::Event &event);

    struct {
        float x;
        float y;
    } button_pressed;
#endif  // AP_SIM_POWERSTACK_SFML_ENABLED

    bool init_done;
};

};  // close namespace SITL

#endif  // AP_SIM_FSO_POWERSTACK_ENABLED
