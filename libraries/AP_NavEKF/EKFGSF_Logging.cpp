#include "EKFGSF_yaw.h"

#include <AP_Logger/AP_Logger.h>

#if HAL_LOGGING_ENABLED

#pragma GCC diagnostic ignored "-Wnarrowing"

void EKFGSF_yaw::Log_Write(uint64_t time_us, LogMessages id0, LogMessages id1, uint8_t core_index)
{
    if (!vel_fuse_running) {
        return;
    }

    static_assert(N_MODELS_EKFGSF >= 5, "Logging will break on <5 EKFGSF models");

    const struct log_KY0 ky0{
        LOG_PACKET_HEADER_INIT(id0),
        time_us                 : time_us,
        core                    : core_index,
        yaw_composite           : float(wrap_360(degrees(GSF.yaw))),
        yaw_composite_variance  : float(sqrtF(MAX(degrees(GSF.yaw_variance), ftype(0.0)))),
        yaw0                    : float(wrap_360(degrees(EKF[0].X[2]))),
        yaw1                    : float(wrap_360(degrees(EKF[1].X[2]))),
        yaw2                    : float(wrap_360(degrees(EKF[2].X[2]))),
        yaw3                    : float(wrap_360(degrees(EKF[3].X[2]))),
        yaw4                    : float(wrap_360(degrees(EKF[4].X[2]))),
        wgt0                    : float(GSF.weights[0]),
        wgt1                    : float(GSF.weights[1]),
        wgt2                    : float(GSF.weights[2]),
        wgt3                    : float(GSF.weights[3]),
        wgt4                    : float(GSF.weights[4]),
    };
    AP::logger().WriteBlock(&ky0, sizeof(ky0));

    const struct log_KY1 ky1{
        LOG_PACKET_HEADER_INIT(id1),
        time_us                 : time_us,
        core                    : core_index,
        ivn0                    : float(EKF[0].innov[0]),
        ivn1                    : float(EKF[1].innov[0]),
        ivn2                    : float(EKF[2].innov[0]),
        ivn3                    : float(EKF[3].innov[0]),
        ivn4                    : float(EKF[4].innov[0]),
        ive0                    : float(EKF[0].innov[1]),
        ive1                    : float(EKF[1].innov[1]),
        ive2                    : float(EKF[2].innov[1]),
        ive3                    : float(EKF[3].innov[1]),
        ive4                    : float(EKF[4].innov[1]),
    };
    AP::logger().WriteBlock(&ky1, sizeof(ky1));
}

#endif  // HAL_LOGGING_ENABLED
