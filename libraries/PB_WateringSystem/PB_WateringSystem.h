#pragma once

#include <AP_BoardConfig/AP_BoardConfig.h>

#ifndef HAL_WATERINGSYSTEM_ENABLED
#define HAL_WATERINGSYSTEM_ENABLED (CONFIG_HAL_BOARD == HAL_BOARD_SITL)
#endif

#if HAL_WATERINGSYSTEM_ENABLED

class PB_WateringSystem {
public:
    void init();
    void update();

private:

    void setup_flowmeter_interrupt();
    void send_serial_from_timings_ringbuffer();

    AP_HAL::UARTDriver *uart;

    void irq_handler(uint8_t pin, bool pin_state, uint32_t timestamp);
    struct IrqState {
        uint32_t last_pulse_us;
    };
    static struct IrqState irq_state;
    bool interrupt_attached;

    uint32_t last_heartbeat_ms;
    const uint16_t HEARTBEAT_INTERVAL_MS = 10000;
    void send_heartbeat();
    void send_debug(const char *fmt, ...) const;
    bool send_message(char type, const char *fmt, ...) const;
    bool send_message(char type, const char *fmt, va_list ap) const;

    void handle_uart_read();

    void command_gpio_ton(char * apin_and_atimeout);
    void command_gpio_in(const uint8_t pin);
    void command_gpio_out(const uint8_t pin);
    void command_gpio_poll(const uint8_t pin);
    void command_gpio_int(const uint8_t pin);
    void command_gpio_ont(const uint8_t pin);
    void command_gpio_on(const uint8_t pin);
    void command_gpio_off(const uint8_t pin);
    void handle_readbuffer_gpio(char *gpio_arguments);

    void handle_readbuffer_version_request(char *gpio_arguments);
    void handle_readbuffer_serial_number_request(char *gpio_arguments);
    void handle_readbuffer_reset_request(char *gpio_arguments);

    static const uint8_t PROTOCOL_VERSION = 5;

    uint8_t map_pin(uint8_t pin);

    char readbuffer[128];
    uint8_t readbuffer_used;

    void check_pin_timeout();
    uint8_t pin_in_timeout;
    uint16_t pin_timeout;  // seconds
    uint32_t pin_timeout_start_ms;

    ObjectBuffer<uint16_t> timings{1024};
    uint16_t max_timings_count;
    uint32_t max_timings_count_emitted_ms;
    uint32_t max_timings_count_emit_ms = 60000;
};

#endif
