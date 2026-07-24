#pragma once

#include <AP_HAL/AP_HAL_Boards.h>
#include <AP_Filesystem/AP_Filesystem_config.h>

// server for the uavcan.protocol.file DroneCAN services, allowing
// files (e.g. dataflash logs) to be listed and downloaded from this
// node by other nodes on the bus
#ifndef AP_DRONECAN_FILE_SERVER_ENABLED
#define AP_DRONECAN_FILE_SERVER_ENABLED (HAL_ENABLE_DRONECAN_DRIVERS && AP_FILESYSTEM_FILE_WRITING_ENABLED && (HAL_PROGRAM_SIZE_LIMIT_KB > 1024))
#endif
