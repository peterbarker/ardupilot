#include <AP_Logger/AP_Logger_config.h>

#if HAL_LOGGING_ENABLED

#include "AC_PosControl.h"

#include <AP_Logger/AP_Logger.h>
#include "LogStructure.h"

// a convenience function for writing out the position controller PIDs
void AC_PosControl::Write_PSCx(LogMessages id, float pos_target, float pos, float vel_desired, float vel_target, float vel, float accel_desired, float accel_target, float accel)
{
    const struct log_PSCx pkt{
        LOG_PACKET_HEADER_INIT(id),
            time_us         : AP_HAL::micros64(),
            pos_target    : pos_target * 0.01f,
            pos           : pos * 0.01f,
            vel_desired   : vel_desired * 0.01f,
            vel_target    : vel_target * 0.01f,
            vel           : vel * 0.01f,
            accel_desired : accel_desired * 0.01f,
            accel_target  : accel_target * 0.01f,
            accel         : accel * 0.01f
    };
    AP::logger().WriteBlock(&pkt, sizeof(pkt));
}

void AC_PosControl::Write_PSCN(float pos_target, float pos, float vel_desired, float vel_target, float vel, float accel_desired, float accel_target, float accel)
{
    Write_PSCx(LOG_PSCN_MSG, pos_target, pos, vel_desired, vel_target, vel, accel_desired, accel_target, accel);
}

void AC_PosControl::Write_PSCE(float pos_target, float pos, float vel_desired, float vel_target, float vel, float accel_desired, float accel_target, float accel)
{
    Write_PSCx(LOG_PSCE_MSG, pos_target, pos, vel_desired, vel_target, vel, accel_desired, accel_target, accel);
}

void AC_PosControl::Write_PSCD(float pos_target, float pos, float vel_desired, float vel_target, float vel, float accel_desired, float accel_target, float accel)
{
    Write_PSCx(LOG_PSCD_MSG, pos_target, pos, vel_desired, vel_target, vel, accel_desired, accel_target, accel);
}

// a convenience function for writing out the position controller PIDs
void AC_PosControl::Write_PSCO(const Vector2p& pos_target_offset_ne, const Vector2p& pos_offset_ne,
                               const Vector2f& vel_target_offset_ne, const Vector2f& vel_offset_ne,
                               const Vector2f& accel_target_offset_ne, const Vector2f& accel_offset_ne)
{
    const struct log_PSCO pkt{
        LOG_PACKET_HEADER_INIT(LOG_PSCO_MSG),
            time_us                   : AP_HAL::micros64(),
            pos_target_offset_north   : pos_target_offset_ne.tofloat().x * 0.01f,
            pos_offset_north          : pos_offset_ne.tofloat().x * 0.01f,
            vel_target_offset_north   : vel_target_offset_ne.x * 0.01f,
            vel_offset_north          : vel_offset_ne.x * 0.01f,
            accel_target_offset_north : accel_target_offset_ne.x * 0.01f,
            accel_offset_north        : accel_offset_ne.x * 0.01f,
            pos_target_offset_east    : pos_target_offset_ne.tofloat().y * 0.01f,
            pos_offset_east           : pos_offset_ne.tofloat().y * 0.01f,
            vel_target_offset_east    : vel_target_offset_ne.y * 0.01f,
            vel_offset_east           : vel_offset_ne.y * 0.01f,
            accel_target_offset_east  : accel_target_offset_ne.y  * 0.01f,
            accel_offset_east         : accel_offset_ne.y * 0.01f
    };
    AP::logger().WriteBlock(&pkt, sizeof(pkt));
}

#endif  // HAL_LOGGING_ENABLED
