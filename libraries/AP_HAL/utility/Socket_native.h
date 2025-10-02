/*
  variant of SocketAPM using native sockets (not via lwip)
 */
#pragma once

#include <AP_HAL/AP_HAL.h>

#if CONFIG_HAL_BOARD == HAL_BOARD_SITL || CONFIG_HAL_BOARD == HAL_BOARD_LINUX
#define AP_SOCKET_NATIVE_ENABLED 1
#define SOCKET_CLASS_NAME SocketAPM_native
#include "Socket.hpp"
#else
#define AP_SOCKET_NATIVE_ENABLED 0
#endif

#undef SOCKET_CLASS_NAME



