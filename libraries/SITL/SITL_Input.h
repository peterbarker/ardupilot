#pragma once

#include <stdint.h>

#include "SITL_config.h"

/*
  structure passed in giving servo positions as PWM values in
  microseconds
*/
struct sitl_input {
    uint16_t servos[SITL_NUM_CHANNELS];
    struct {
        float speed;      // m/s
        float direction;  // degrees 0..360
        float turbulence;
        float dir_z;	  //degrees -90..90
    } wind;
};

