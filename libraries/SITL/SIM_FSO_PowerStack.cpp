#include "SIM_config.h"

#if AP_SIM_FSO_POWERSTACK_ENABLED

#include "SIM_FSO_PowerStack.h"

#include <AP_HAL/HAL.h>

#if AP_SIM_POWERSTACK_SFML_ENABLED
#ifdef HAVE_SFML_GRAPHICS_H
#include <SFML/Graphics.h>
#else
#include <SFML/Graphics.hpp>
#endif
#endif  // AP_SIM_POWERSTACK_SFML_ENABLED


#include <SITL/SITL.h>

#include <AP_Notify/AP_Notify.h>

#include "SIM_INA2xx.h"
#include "SIM_TI_DACx3204.h"

using namespace SITL;

#if AP_SIM_POWERSTACK_SFML_ENABLED

void FSO_PowerStack::update_thread(void)
{
    sf::RenderWindow *w = nullptr;
    {
#if AP_NOTIFY_ENABLED
        WITH_SEMAPHORE(AP::notify().sf_window_mutex);
#endif
        w = new sf::RenderWindow(sf::VideoMode(width, height), "PowerStack");
    }

    if (w == nullptr) {
        AP_HAL::panic("Unable to create SIM_FSO_PowerStack window");
    }

    const char *font_filepath = "/usr/share/fonts/truetype/ubuntu/Ubuntu-B.ttf";

    // initialise GUI stuff:
    sf::Font font;
    font.loadFromFile(font_filepath);

    char v0_string[20];
    snprintf(v0_string, ARRAY_SIZE(v0_string), "v0: ??");
    sf::Text v0_text;
    v0_text.setString("v0: ??");
    v0_text.setFont(font);
    v0_text.setCharacterSize(20);
    v0_text.setFillColor(sf::Color(128, 128, 0));
    v0_text.setOutlineColor(sf::Color(128, 128, 0));

    std::vector<sf::Drawable*> elements;
    elements.push_back(&v0_text);

    while (true) {
        {
#if AP_NOTIFY_ENABLED
            WITH_SEMAPHORE(AP::notify().sf_window_mutex);
#endif
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

            static float last_v0 = -1;
            const float v0 = dac->get_dac(0)->get_voltage();
            bool need_redisplay = false;
            if (!is_equal(v0, last_v0)) {
                last_v0 = v0;
                need_redisplay = true;
                snprintf(v0_string, ARRAY_SIZE(v0_string), "v0: %.02f", v0);
                v0_text.setString(v0_string);
            }

            if (need_redisplay) {
                w->clear(sf::Color(255,255,255));
                for (auto *element : elements) {
                    w->draw(*element);
                }
                w->display();
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
#endif  // AP_SIM_POWERSTACK_SFML_ENABLED


void FSO_PowerStack::init()
{
#if AP_SIM_POWERSTACK_SFML_ENABLED
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
