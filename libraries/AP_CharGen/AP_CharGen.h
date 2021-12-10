#pragma once

#include <AP_HAL/AP_HAL_Boards.h>

#ifndef AP_CHARGEN_ENABLED
#define AP_CHARGEN_ENABLED 1
#endif


#if AP_CHARGEN_ENABLED

class AP_CharGen
{
public:
    AP_CharGen() { }

    void update();

private:

    static const uint8_t MAX_UARTS { 4 };
    AP_HAL::UARTDriver *uarts[MAX_UARTS];
    uint16_t uart_buffer_offsets[MAX_UARTS];

    // update the uarts we send data to periodically.  Returns true if
    // any uarts are configured
    bool config_check();
    uint32_t last_config_check;

    // buffer is dynamically allocated and contains the chargen output
    // repeated many times.  This ensures we don't burn vast amounts
    // of CPU writing into temporary buffers
    uint8_t *buffer;
    uint16_t buflen;

    // fill the buffer with the chargen output
    void init_buffer();

    uint16_t short_writes;
};

#endif
