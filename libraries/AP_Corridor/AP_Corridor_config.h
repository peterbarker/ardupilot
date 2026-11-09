#pragma once

#include <AP_HAL/AP_HAL_Boards.h>

// the corridor graph mission type is work-in-progress in the MAVLink
// definitions, so it is only enabled by default in SITL for now
#ifndef AP_CORRIDOR_ENABLED
#if defined(HAL_BUILD_AP_PERIPH)
#define AP_CORRIDOR_ENABLED 0
#else
#define AP_CORRIDOR_ENABLED (CONFIG_HAL_BOARD == HAL_BOARD_SITL)
#endif
#endif

// maximum number of items (nodes plus edges) in a corridor graph
#ifndef AP_CORRIDOR_MAX_ITEMS
#define AP_CORRIDOR_MAX_ITEMS 500
#endif
