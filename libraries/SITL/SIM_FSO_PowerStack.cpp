#include "SIM_config.h"

#if AP_SIM_FSO_POWERSTACK_ENABLED

#include "SIM_FSO_PowerStack.h"

#include <AP_HAL/HAL.h>

#ifdef HAVE_SFML_GRAPHICS_H
#include <SFML/Graphics.h>
#else
#include <SFML/Graphics.hpp>
#endif

#include <SITL/SITL.h>

#include <AP_Notify/AP_Notify.h>

#include "SIM_INA2xx.h"
#include "SIM_TI_DACx3204.h"

using namespace SITL;

void FSO_PowerStack::update_thread(void)
{
    sf::RenderWindow *w = nullptr;
    {
        WITH_SEMAPHORE(AP::notify().sf_window_mutex);
        w = new sf::RenderWindow(sf::VideoMode(width, height), "PowerStack");
    }

    if (w == nullptr) {
        AP_HAL::panic("Unable to create SIM_FSO_PowerStack window");
    }

    while (true) {
        {
            WITH_SEMAPHORE(AP::notify().sf_window_mutex);
            sf::Event event;
            while (w->pollEvent(event)) {
                if (event.type == sf::Event::Closed) {
                    w->close();
                    break;
                }
            }
            if (!w->isOpen()) {
                break;
            }
            // const uint32_t colour = red<<16 | green<<8 | blue;
            // if (colour != last_colour) {
            //     last_colour = colour;
            //     w->clear(sf::Color(red, green, blue, 255));
            //     w->display();
            // }
        }
        usleep(10000);
    }
}

// trampoline for update thread
void *FSO_PowerStack::update_thread_start(void *obj)
{
    ((FSO_PowerStack *)obj)->update_thread();
    return nullptr;
}

void FSO_PowerStack::init()
{
#ifdef WITH_SITL_FSO_PowerStack
    pthread_create(&thread, NULL, update_thread_start, this);
#endif
}

void FSO_PowerStack::update(Aircraft &aircraft)
{
    const auto &sitl = AP::sitl();
    // toggle fan pin...
    const auto now = AP_HAL::micros();
    if (now - last_fan_toggle_us > 5000) {
        last_fan_toggle_us = now;
        sitl->full_pin_mask.toggle(200);
    }

    ina[0].set_simulated_voltage(dac->get_dac(0)->get_voltage());
}

#endif  // AP_SIM_FSO_POWERSTACK_ENABLED
