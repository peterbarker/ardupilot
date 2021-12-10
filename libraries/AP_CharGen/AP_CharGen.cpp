#include "AP_CharGen.h"

#if AP_CHARGEN_ENABLED

#include <AP_Math/AP_Math.h>
#include <AP_SerialManager/AP_SerialManager.h>


// TODO:
//  - option to emit on thread rather than main thread
//  - option for buffered/unbuffered writes
//  - option to set buffer sizes
//  - option to read and report on bytes-read
//  - option to force off hardware flow control?

void AP_CharGen::update()
{
    // check to see if the user's manipulated parameters to enable or
    // disable chargen output on specific UARTS:
    const bool some_uarts_configured = config_check();
    if (!some_uarts_configured) {
        if (buffer != nullptr) {
            free(buffer);
            buffer = nullptr;
        }
        return;
    }

    if (buffer == nullptr) {
        init_buffer();
        if (buffer == nullptr) {
            return;
        }
    }

    // emit chargen data to each configured UART:
    for (uint8_t i=0; i<ARRAY_SIZE(uarts); i++) {
        AP_HAL::UARTDriver *uart = uarts[i];
        if (uart == nullptr) {
            break;
        }
        uint32_t space = uart->txspace();
        while (space != 0) {
            const uint16_t ofs = uart_buffer_offsets[i];
            const uint16_t to_write = MIN(space, unsigned(buflen-ofs));
            const uint32_t written = uart->write(&buffer[ofs], to_write);
            if (written < to_write) {
                short_writes++;
            }
            if (written == 0) {
                break;
            }
            space -= written;
            uart_buffer_offsets[i] += written;
            if (uart_buffer_offsets[i] >= buflen) {
                uart_buffer_offsets[i] = 0;
            }
        }
    }
}


bool AP_CharGen::config_check()
{
    const AP_SerialManager &sm = AP::serialmanager();

    uint8_t i;
    for (i = 0; i<ARRAY_SIZE(uarts); i++) {
        AP_HAL::UARTDriver *uart = sm.find_serial(AP_SerialManager::SerialProtocol_CharGen, i);
        if (uart == nullptr) {
            break;
        }
        if (uarts[i] != uart) {
            uarts[i] = uart;
            uart_buffer_offsets[i] = 0;
        }
    }
    // remove any duplicates / spares
    for (; i<ARRAY_SIZE(uarts); i++) {
        uarts[i] = nullptr;
    }
    return i != 0;
}

void AP_CharGen::init_buffer()
{
    const uint32_t now = AP_HAL::millis();

    // only check once/second:
    if (last_config_check - now < 1000) {
        return;
    }

    last_config_check = now;

    const uint8_t line_length = 74;  // 72 + crlf
    const uint8_t num_lines = 72;

    buffer = (uint8_t*)malloc(line_length * num_lines);

    if (buffer == nullptr) {
        return;
    }

    for (auto line=0; line<num_lines; line++) {
        for (uint8_t i=0; i<72; i++) {
            uint8_t c = ' ' + line + i;
            if (c >= ' ' + 72) {
                c -= 72;
            }
            buffer[buflen++] = c;
        }
        buffer[buflen++] = '\r';
        buffer[buflen++] = '\n';
    }
}

#endif
