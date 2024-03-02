#include "AP_InternalError.h"

#include <AP_HAL/HAL.h>
#include <AP_HAL/Util.h>

extern const AP_HAL::HAL &hal;

#include <stdio.h>

// stack overflow hook for low level RTOS code, C binding
void AP_stack_overflow(const char *thread_name)
{
    static bool done_stack_overflow;
    INTERNAL_ERROR(AP_InternalError::error_t::stack_overflow);
    if (!done_stack_overflow) {
        // we don't want to record the thread name more than once, as
        // first overflow can trigger a 2nd
        strncpy_noterm(hal.util->persistent_data.thread_name4, thread_name, 4);
        done_stack_overflow = true;
    }
    hal.util->persistent_data.fault_type = 42; // magic value
    if (!hal.util->get_soft_armed()) {
        AP_HAL::panic("stack overflow %s\n", thread_name);
    }
}

// hook for memory guard errors with --enable-memory-guard
void AP_memory_guard_error(uint32_t size)
{
    INTERNAL_ERROR(AP_InternalError::error_t::mem_guard);
    if (!hal.util->get_soft_armed()) {
        ::printf("memory guard error size=%u\n", unsigned(size));
        AP_HAL::panic("memory guard size=%u\n", unsigned(size));
    }
}
