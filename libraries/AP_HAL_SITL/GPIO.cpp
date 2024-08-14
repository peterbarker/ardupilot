
#include "GPIO.h"

#if CONFIG_HAL_BOARD == HAL_BOARD_SITL

using namespace HALSITL;

extern const AP_HAL::HAL& hal;

#define SITL_WOW_ALTITUDE 0.01

void GPIO::init()
{}

void GPIO::pinMode(uint8_t pin, uint8_t output)
{
    if (!valid_pin(pin)) {
        return;
    }
    if (output) {
        pin_mode_is_write.set(pin);
    } else {
        pin_mode_is_write.clear(pin);
    }
}

uint8_t GPIO::read(uint8_t pin)
{
    const auto *sitl = _sitlState->_sitl;
    if (sitl == nullptr) {
        return 0;
    }

    if (!valid_pin(pin)) {
        return 0;
    }

    // weight on wheels pin support
    if (pin == sitl->wow_pin.get()) {
        return sitl->state.altitude < SITL_WOW_ALTITUDE ? 1 : 0;
    }

    return sitl->full_pin_mask.get(pin);
}

void GPIO::write(uint8_t pin, uint8_t value)
{
    auto *sitl = _sitlState->_sitl;
    if (sitl == nullptr) {
        return;
    }

    if (!valid_pin(pin)) {
        return;
    }

    // weight-on-wheels pin support:
    if (pin == sitl->wow_pin.get()) {
        return;
    }

    if (!(pin_mode_is_write.get(pin))) {
        // ignore setting of pull-up resistors
        return;
    }

    if (value) {
        sitl->full_pin_mask.set(pin);
    } else {
        sitl->full_pin_mask.clear(pin);
    }
    // also update the parameter used to provide external visibility
    // for the first 16 bits:
    uint16_t mask = static_cast<uint16_t>(_sitlState->_sitl->pin_mask.get());
    uint16_t new_mask = mask;

    if (value) {
        new_mask |= (1U << pin);
    } else {
        new_mask &= ~(1U << pin);
    }
    if (mask != new_mask) {
        _sitlState->_sitl->pin_mask.set_and_notify(new_mask);
    }
}

void GPIO::toggle(uint8_t pin)
{
    write(pin, !read(pin));
}

/* Alternative interface: */
AP_HAL::DigitalSource* GPIO::channel(uint16_t n) {
    if (valid_pin(n)) {
        return NEW_NOTHROW DigitalSource(static_cast<uint8_t>(n));
    } else {
        return nullptr;
    }
    return new DigitalSource(static_cast<uint8_t>(n));
}

bool GPIO::usb_connected(void)
{
    return false;
}

DigitalSource::DigitalSource(uint8_t pin) :
    _pin(pin)
{}

void DigitalSource::mode(uint8_t output)
{}

uint8_t DigitalSource::read()
{
    return hal.gpio->read(_pin);
}

void DigitalSource::write(uint8_t value)
{
    value = static_cast<uint8_t>(value ? 1 : 0);
    return hal.gpio->write(_pin, value);
}

void DigitalSource::toggle()
{
    return hal.gpio->write(_pin, !hal.gpio->read(_pin));
}
#endif
