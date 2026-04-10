#pragma once

#include <AP_HAL/AP_HAL_Boards.h>

// Frame types
#define UNDEFINED_FRAME 0
#define MULTICOPTER_FRAME 1
#define HELI_FRAME 2

// Airmode
enum class AirMode {
    AIRMODE_NONE,
    AIRMODE_DISABLED,
    AIRMODE_ENABLED,
};

// bit options for DEV_OPTIONS parameter
enum DevOptions {
    DevOptionADSBMAVLink = 1,
    DevOptionVFR_HUDRelativeAlt = 2,
};

//  Logging parameters - only 32 messages are available to the vehicle here.
enum LoggingParameters {
     LOG_CONTROL_TUNING_MSG,
     LOG_DATA_INT16_MSG,
     LOG_DATA_UINT16_MSG,
     LOG_DATA_INT32_MSG,
     LOG_DATA_UINT32_MSG,
     LOG_DATA_FLOAT_MSG,
     LOG_PARAMTUNE_MSG,
     LOG_HELI_MSG,
     LOG_GUIDED_POSITION_TARGET_MSG,
     LOG_SYSIDD_MSG,
     LOG_SYSIDS_MSG,
     LOG_GUIDED_ATTITUDE_TARGET_MSG,
     LOG_RATE_THREAD_DT_MSG
};

#define MASK_LOG_ATTITUDE_FAST          (1<<0)
#define MASK_LOG_ATTITUDE_MED           (1<<1)
#define MASK_LOG_GPS                    (1<<2)
#define MASK_LOG_PM                     (1<<3)
#define MASK_LOG_CTUN                   (1<<4)
#define MASK_LOG_NTUN                   (1<<5)
#define MASK_LOG_RCIN                   (1<<6)
#define MASK_LOG_IMU                    (1<<7)
#define MASK_LOG_CMD                    (1<<8)
#define MASK_LOG_CURRENT                (1<<9)
#define MASK_LOG_RCOUT                  (1<<10)
#define MASK_LOG_OPTFLOW                (1<<11)
#define MASK_LOG_PID                    (1<<12)
#define MASK_LOG_COMPASS                (1<<13)
#define MASK_LOG_INAV                   (1<<14) // deprecated
#define MASK_LOG_CAMERA                 (1<<15)
#define MASK_LOG_MOTBATT                (1UL<<17)
#define MASK_LOG_IMU_FAST               (1UL<<18)
#define MASK_LOG_IMU_RAW                (1UL<<19)
#define MASK_LOG_VIDEO_STABILISATION    (1UL<<20)
#define MASK_LOG_FTN_FAST               (1UL<<21)
#define MASK_LOG_ANY                    0xFFFF

// for PILOT_THR_BHV parameter
#define THR_BEHAVE_FEEDBACK_FROM_MID_STICK (1<<0)
#define THR_BEHAVE_HIGH_THROTTLE_CANCELS_LAND (1<<1)
#define THR_BEHAVE_DISARM_ON_LAND_DETECT (1<<2)
