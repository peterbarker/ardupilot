#include "PB_WateringSystem.h"

#if HAL_WATERINGSYSTEM_ENABLED

#include <AP_SerialManager/AP_SerialManager.h>

#include <stdarg.h>

#include <AP_Math/AP_Math.h>

extern const AP_HAL::HAL& hal;

struct PB_WateringSystem::IrqState PB_WateringSystem::irq_state;

void PB_WateringSystem::init()
{
    uart = AP::serialmanager().find_serial(AP_SerialManager::SerialProtocol_PBWateringSystem, 0);
    if (uart == nullptr) {
        return;
    }

    send_message('I', "Hello");

    setup_flowmeter_interrupt();
}

void PB_WateringSystem::setup_flowmeter_interrupt()
{
    const uint8_t somepin = 61;
    hal.gpio->attach_interrupt(
        somepin,
        FUNCTOR_BIND_MEMBER(&PB_WateringSystem::irq_handler, void, uint8_t, bool, uint32_t),
        AP_HAL::GPIO::INTERRUPT_RISING);
}

void PB_WateringSystem::irq_handler(uint8_t pin, bool pin_state, uint32_t timestamp)
{
    const uint32_t dt = timestamp - irq_state.last_pulse_us;
    irq_state.last_pulse_us = timestamp;
    // we don't accept pulses less than 100us. Using an irq for such
    // high RPM is too inaccurate, and it is probably just bounce of
    // the signal which we should ignore
    if (dt < 100) {
        return;
    }
    if (dt > 1000*1000) {
        return;
    }
    timings.push(dt/1000.0);
}

uint8_t PB_WateringSystem::map_pin(uint8_t pin)
{
    if (pin > 49) {
        return pin;
    }
    switch (pin) {
    case 3 ... 11:
        return pin + 47;
    case 14 ... 15:
        return pin + 45;
    case 16:
        return 1;
    }
    send_debug("No map for %u", pin);
    return 50;
}


void PB_WateringSystem::check_pin_timeout()
{
    if (pin_timeout_start_ms == 0) {
        return;
    }
    if (AP_HAL::millis() - pin_timeout_start_ms < pin_timeout * 1000) {
        return;
    }

    // timed out
    pin_timeout_start_ms = 0;
    pin_timeout = 0;
    const uint8_t somepin = map_pin(pin_in_timeout);
    hal.gpio->write(somepin, 1);
    send_message('G', "TO %u", pin_in_timeout);
}

void PB_WateringSystem::command_gpio_ton(char * apin_and_atimeout)
{
    send_debug("ton args: %s", apin_and_atimeout);
    if (pin_timeout != 0) {
        send_debug("pin in timeout already");
        return;
    }

    char *space = strchr(apin_and_atimeout + 1,' ');
    if (space == nullptr) {
        send_debug("malformed");
        return;
    }
    *space = '\0';
    char *arguments = space + 1;
    pin_in_timeout = atoi(apin_and_atimeout);
    pin_timeout = atoi(arguments);
    pin_timeout_start_ms = AP_HAL::millis();

    const uint8_t somepin = map_pin(pin_in_timeout);
    send_debug("setting pin %u on", somepin);
    hal.gpio->write(somepin, 0);
    send_message('G',"TON %d", pin_in_timeout);
}
void PB_WateringSystem::command_gpio_in(const uint8_t pin)
{
    const uint8_t somepin = map_pin(pin);
    hal.gpio->pinMode(somepin, 0);
    send_message('G',"IN");
}
void PB_WateringSystem::command_gpio_out(const uint8_t pin)
{
    const uint8_t somepin = map_pin(pin);
    hal.gpio->pinMode(somepin, 1);
    send_message('G',"OUT");
}
void PB_WateringSystem::command_gpio_poll(const uint8_t pin)
{
    const uint8_t somepin = map_pin(pin);
    send_message('G',"P %u",hal.gpio->read(somepin));
}
void PB_WateringSystem::command_gpio_int(const uint8_t pin)
{
    if (interrupt_attached) {
        send_debug("int already attached");
        return;
    }
    const uint8_t somepin = map_pin(pin);
    hal.gpio->attach_interrupt(somepin,
                               FUNCTOR_BIND_MEMBER(&PB_WateringSystem::irq_handler, void, uint8_t, bool, uint32_t),
                               AP_HAL::GPIO::INTERRUPT_RISING);
    interrupt_attached = true;
    // setInputPullup(&somepin);
    send_message('G',"INT");
}
void PB_WateringSystem::command_gpio_ont(const uint8_t pin)
{
    if (!interrupt_attached) {
        send_debug("int not attached");
        return;
    }
    const uint8_t somepin = map_pin(pin);
    hal.gpio->detach_interrupt(somepin);
    interrupt_attached = false;
    send_message('G',"ONT");
}
void PB_WateringSystem::command_gpio_on(const uint8_t pin)
{
    uint8_t somepin = map_pin(pin);
    hal.gpio->write(somepin, 1);
    send_message('G',"ON");
}
void PB_WateringSystem::command_gpio_off(const uint8_t pin)
{
    uint8_t somepin = map_pin(pin);
    hal.gpio->write(somepin, 0);
    send_message('G',"OFF");
}


void PB_WateringSystem::handle_readbuffer_gpio(char *command)
{
    if (!strncmp(command,"TON",3)) {
      command_gpio_ton(&command[4]);
      return;
    }
    char *space = strchr(command, ' ');
    if (space == nullptr) {
        send_debug("No space");
        return;
    }
    *space = '\0';
    char *pin_string = space + 1;
    const uint8_t pin = atoi(pin_string);
    if (!strncmp(command,"OUT",3)) {
        command_gpio_out(pin);
    } else if (!strncmp(command,"POLL",4)) {
        command_gpio_poll(pin);
    } else if (!strncmp(command,"INT",3)) {
        command_gpio_int(pin);
    } else if (!strncmp(command,"ONT",3)) {
        command_gpio_ont(pin);
    } else if (!strncmp(command,"IN",2)) {
        command_gpio_in(pin);
    } else if (!strncmp(command,"ON",2)) {
        command_gpio_on(pin);
    } else if (!strncmp(command,"OFF",3)) {
        command_gpio_off(pin);
    } else {
        send_debug("%s?",command);
    }
}

void PB_WateringSystem::handle_readbuffer_version_request(char *gpio_arguments)
{
    send_message('V', "%u", PROTOCOL_VERSION);
}

void PB_WateringSystem::handle_readbuffer_serial_number_request(char *gpio_arguments)
{
    send_message('S', "%u", 2);  // parameter?
}

void PB_WateringSystem::handle_readbuffer_reset_request(char *gpio_arguments)
{
    hal.scheduler->reboot(false);
}

void PB_WateringSystem::handle_uart_read()
{
    ssize_t bytes_read = uart->read((uint8_t*)&readbuffer[readbuffer_used], ARRAY_SIZE(readbuffer) - readbuffer_used);
    if (bytes_read > 0) {
        readbuffer_used += bytes_read;
    }
    while (true) {
        void *cr = memchr(readbuffer, '\r', readbuffer_used);
        if (cr == nullptr) {
            return;
        }
        *(char*)cr = '\0';
        send_debug("C:%s", readbuffer);
        switch (readbuffer[0]) {
        case 'G':
            handle_readbuffer_gpio(&readbuffer[2]);
            break;
        case 'V':
            handle_readbuffer_version_request(&readbuffer[2]);
            break;
        case 'S':
            handle_readbuffer_serial_number_request(&readbuffer[2]);
            break;
        case 'R':
            handle_readbuffer_reset_request(&readbuffer[2]);
            break;
        default:
            send_debug("Unknown: %s", readbuffer);
        }
        // consume the bytes from the readbuffer:
        const uint16_t to_consume = (char*)cr - readbuffer + 1;
        const uint16_t remaining_byte_count = readbuffer_used - to_consume;
        memmove(readbuffer, cr, remaining_byte_count);
        readbuffer_used = remaining_byte_count;
    }
}

void PB_WateringSystem::send_serial_from_timings_ringbuffer()
{
    while (!timings.is_empty()) {
        if (uart->txspace() < 10) {
            break;
        }
        max_timings_count = MAX(max_timings_count, timings.available());
        uint16_t timing;
        if (!timings.pop(timing)) {
            // unexpected....
            break;
        }
        send_message('T', "%u", timing);
    }
}

void PB_WateringSystem::update()
{
    if (uart == nullptr) {
        return;
    }

    // handle incoming commands
    handle_uart_read();

    // check for pin timeout
    check_pin_timeout();

    // consider sending a heartbeat
    const uint32_t now = AP_HAL::millis();
    if (now - last_heartbeat_ms > HEARTBEAT_INTERVAL_MS) {
        send_heartbeat();
        last_heartbeat_ms = now;
    }

    // send stuff from the timings ringbuffer
    send_serial_from_timings_ringbuffer();

    // send max timings count diagnostics
    if (now - max_timings_count_emitted_ms > max_timings_count_emit_ms) {
        max_timings_count_emitted_ms = now;
        send_debug("mtc=%u", max_timings_count);
    }

}

void PB_WateringSystem::send_heartbeat()
{
    send_message('H', "%u", AP_HAL::millis() / 1000);
}
void PB_WateringSystem::send_debug(const char *fmt, ...) const
{
    va_list ap;
    va_start(ap, fmt);
    send_message('D', fmt, ap);
    va_end(ap);
}

bool PB_WateringSystem::send_message(char type, const char *fmt, ...) const
{
    va_list ap;
    va_start(ap, fmt);
    bool ret = send_message(type, fmt, ap);
    va_end(ap);
    return ret;

}
bool PB_WateringSystem::send_message(char type, const char *fmt, va_list ap) const
{
    char newfmt[128];
    hal.util->snprintf(newfmt, ARRAY_SIZE(newfmt), "%c %s %c\r\n", type, fmt, type + 0x20);

    uint8_t buffer[128];
    uint16_t len = hal.util->vsnprintf((char*)buffer, ARRAY_SIZE(buffer), newfmt, ap);

    if (uart->txspace() < len) {
        return false;
    }

    if (uart->write(buffer, len)) {
        return false;
    }

    return true;
}

#endif
