#include "SIM_config.h"

#if AP_SIM_FSO_POWERSTACK_ENABLED

#include "SIM_FSO_PowerStack.h"

#include <AP_HAL/HAL.h>

#include <GCS_MAVLink/GCS.h>

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
FSO_PowerStack::Button FSO_PowerStack::buttons[] {
    { "Main", 70, 10, 10 },
    { "Payload", 85, 10, 10 },
};

const FSO_PowerStack::Button *FSO_PowerStack::find_button_for_MouseButton(const sf::Event &event)
{
    const float ev_x = event.mouseButton.x;
    const float ev_y = event.mouseButton.y;
    for (const auto &b : buttons) {
        const Vector2f delta {
            b.posx - ev_x,
            b.posy - ev_y
        };
        if (delta.length() < b.diameter/2) {
            return &b;
        }
    }
    return nullptr;
}

void FSO_PowerStack::handle_MouseButtonPressed(const sf::Event &event)
{
}

void FSO_PowerStack::handle_MouseButtonReleased(const sf::Event &event)
{
    auto clicked_button = find_button_for_MouseButton(event);
    if (clicked_button == nullptr) {
        return;
    }
    const auto &tmp = clicked_button;
    clicked_button = nullptr;

    auto b = find_button_for_MouseButton(event);
    if (b == nullptr) {
        return;
    }
    if (b != tmp) {
        return;
    }

    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "Click");
}

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

    static struct {
        const char *label;
        const uint8_t offset;
        const float posx;
        const float posy;
        float voltage;
        sf::Text text;
    } dacs[] {
        { "dac0", 0, 10, 10, -1 },
        { "dac1", 1, 10, 30, -1 },
        { "dac2", 2, 10, 50 -1 },
        { "dac3", 3, 10, 70, -1 },
    };

    std::vector<sf::Drawable*> elements;

    for (auto &adac : dacs) {
        char label_string[20];
        snprintf(label_string, ARRAY_SIZE(label_string), "%s: ??", adac.label);
        adac.text.setString(label_string);
        adac.text.setFont(font);
        adac.text.setCharacterSize(20);
        adac.text.setFillColor(sf::Color(128, 128, 0));
        adac.text.setOutlineColor(sf::Color(128, 128, 0));
        adac.text.setPosition(adac.posx, adac.posy);

        elements.push_back(&adac.text);
    }

    // draw buttons
    for (auto &button : buttons) {
        button.shape.setFillColor(sf::Color(128, 128, 128));
        button.shape.setPosition(button.posx-button.getRadius(), button.posy-button.getRadius());
        elements.push_back(&button.shape);
        // and a label for the circle:
    }

    while (true) {
        {
#if AP_NOTIFY_ENABLED
            WITH_SEMAPHORE(AP::notify().sf_window_mutex);
#endif
            sf::Event event;
            while (w->pollEvent(event)) {
                switch (event.type) {
                case sf::Event::Closed:
                    w->close();
                    break;
                case sf::Event::Resized:
                case sf::Event::LostFocus:
                case sf::Event::GainedFocus:
                case sf::Event::TextEntered:
                case sf::Event::KeyPressed:
                case sf::Event::KeyReleased:
                default:  // I got bored
                    break;
                case sf::Event::MouseButtonPressed:
                    handle_MouseButtonPressed(event);
                    break;
                case sf::Event::MouseButtonReleased:
                    handle_MouseButtonReleased(event);
                    break;
                }
            }
            if (!w->isOpen()) {
                break;
            }

            bool need_redisplay;
            for (auto &adac : dacs) {
                float new_voltage = dac->get_dac(adac.offset)->get_voltage();
                if (is_equal(new_voltage, adac.voltage)) {
                    continue;
                }
                adac.voltage = new_voltage;
                need_redisplay = true;
                char label_string[20];
                snprintf(label_string, ARRAY_SIZE(label_string), "%s: %.02f", adac.label, adac.voltage);
                adac.text.setString(label_string);
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
}

void FSO_PowerStack::update(Aircraft &aircraft)
{
    if (!init_done) {
#if AP_SIM_POWERSTACK_SFML_ENABLED
        // this needs the notify library to be ready:
        pthread_create(&thread, NULL, update_thread_start, this);
#endif
        init_done = true;
    }

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
