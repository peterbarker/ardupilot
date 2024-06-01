#pragma once

#include "defines.h"

#ifndef MAV_SYSTEM_ID
  #define MAV_SYSTEM_ID    1
#endif

//////////////////////////////////////////////////////////////////////////////
// FrSky telemetry support
//

#ifndef CH7_OPTION
  #define CH7_OPTION CH7_SAVE_WP
#endif

//////////////////////////////////////////////////////////////////////////////
// MODE
// MODE_CHANNEL
//
#ifndef MODE_CHANNEL
  #define MODE_CHANNEL    8
#endif
#if (MODE_CHANNEL != 5) && (MODE_CHANNEL != 6) && (MODE_CHANNEL != 7) && (MODE_CHANNEL != 8)
  #error XXX
  #error XXX You must set MODE_CHANNEL to 5, 6, 7 or 8
  #error XXX
#endif

//////////////////////////////////////////////////////////////////////////////
// NAVL1
//
#ifndef NAVL1
  #define NAVL1_PERIOD    8
#endif

//////////////////////////////////////////////////////////////////////////////
// CRUISE_SPEED default
//
#ifndef CRUISE_SPEED
  #define CRUISE_SPEED    2  // in m/s
#endif

#define DEFAULT_LOG_BITMASK    0xffff

//////////////////////////////////////////////////////////////////////////////
// Acro mode - basic stabilised steering mode
#ifndef AP_ROVER_MODE_ACRO_ENABLED
# define AP_ROVER_MODE_ACRO_ENABLED 1
#endif

//////////////////////////////////////////////////////////////////////////////
// Auto mode - waypoint-following mode
#ifndef AP_ROVER_MODE_AUTO_ENABLED
# define AP_ROVER_MODE_AUTO_ENABLED AP_MISSION_ENABLED
#endif

//////////////////////////////////////////////////////////////////////////////
// Circle mode - circle around specifc location
#ifndef AP_ROVER_MODE_CIRCLE_ENABLED
# define AP_ROVER_MODE_CIRCLE_ENABLED 1
#endif

//////////////////////////////////////////////////////////////////////////////
// Dock mode - allows vehicle to dock to a docking target
#ifndef AP_ROVER_MODE_DOCK_ENABLED
# define AP_ROVER_MODE_DOCK_ENABLED AC_PRECLAND_ENABLED
#endif

//////////////////////////////////////////////////////////////////////////////
// Follow mode - allows vehicle to follow target
#ifndef AP_ROVER_MODE_FOLLOW_ENABLED
# define AP_ROVER_MODE_FOLLOW_ENABLED AP_FOLLOW_ENABLED
#endif

//////////////////////////////////////////////////////////////////////////////
// Guided mode - offboard waypoint or attitude control
#ifndef AP_ROVER_MODE_GUIDED_ENABLED
# define AP_ROVER_MODE_GUIDED_ENABLED 1
#endif

//////////////////////////////////////////////////////////////////////////////
// Hold mode - stop vehicle moving under its own power (BalanceBot keeps balancing)
#ifndef AP_ROVER_MODE_HOLD_ENABLED
# define AP_ROVER_MODE_HOLD_ENABLED 1
#endif

#if !AP_ROVER_MODE_HOLD_ENABLED
#error Mode hold must be compiled in for now
#endif

//////////////////////////////////////////////////////////////////////////////
// Loiter mode - hold current position
#ifndef AP_ROVER_MODE_LOITER_ENABLED
# define AP_ROVER_MODE_LOITER_ENABLED 1
#endif

//////////////////////////////////////////////////////////////////////////////
// Manual mode - basic pass-through control of inputs to mixed outputs
#ifndef AP_ROVER_MODE_MANUAL_ENABLED
# define AP_ROVER_MODE_MANUAL_ENABLED 1
#endif

//////////////////////////////////////////////////////////////////////////////
// RTL mode - vehicle travels towards home (or rally items)
#ifndef AP_ROVER_MODE_RTL_ENABLED
# define AP_ROVER_MODE_RTL_ENABLED 1
#endif

//////////////////////////////////////////////////////////////////////////////
// Simple mode - vehicle is controlled via earth-frame rather than body frame RC input
#ifndef AP_ROVER_MODE_SIMPLE_ENABLED
# define AP_ROVER_MODE_SIMPLE_ENABLED 1
#endif

//////////////////////////////////////////////////////////////////////////////
// SmartRTL mode - vehicle retraces its path to home
#ifndef AP_ROVER_MODE_SMARTRTL_ENABLED
# define AP_ROVER_MODE_SMARTRTL_ENABLED 1
#endif

//////////////////////////////////////////////////////////////////////////////
// Steering mode - basic driving mode based on desired lateral acceleration
#ifndef AP_ROVER_MODE_STEERING_ENABLED
# define AP_ROVER_MODE_STEERING_ENABLED 1
#endif

//////////////////////////////////////////////////////////////////////////////
// Developer Items
//

#ifndef AP_ROVER_ADVANCED_FAILSAFE_ENABLED
  #define AP_ROVER_ADVANCED_FAILSAFE_ENABLED 0
#endif

#ifndef AP_ROVER_AUTO_ARM_ONCE_ENABLED
#define AP_ROVER_AUTO_ARM_ONCE_ENABLED 1
#endif
