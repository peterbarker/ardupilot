'''
Copter autotest: GPSForYawCompassFallback

AP_FLAKE8_CLEAN
'''

import math

from pymavlink import mavextra

from vehicle_test_suite import NotAchievedException

from test_discovery import TestGroup
from test_discovery import testcase


class GPSForYawCompassFallback(TestGroup):
    @testcase(bucket="CopterTests1d")
    def GPSForYawCompassFallback(self):
        '''EKF3 must fall back to the compass when GPS yaw is present but unusable'''
        # With EK3_SRC1_YAW = 3 (GPS with compass fallback) the EKF falls
        # back to the magnetometer once no usable GPS yaw has been seen for
        # 10 seconds.  A moving-baseline yaw whose configured antenna offset
        # is vertical carries no yaw information, so the EKF rejects the
        # measurement geometrically instead of fusing it.  A rejected
        # measurement must count as "no usable yaw": the GPS keeps
        # publishing yaw, so if rejection refreshed the last-yaw timestamp
        # the fallback would never engage and the vehicle would fly with no
        # yaw aiding at all.
        self.load_default_params_file("copter-gps-for-yaw.parm")
        self.set_parameters({
            "EK3_SRC1_YAW": 3,  # GPS with compass fallback
        })
        self.reboot_sitl()

        self.wait_gps_fix_type_gte(6, message_type="GPS2_RAW", verbose=True)
        self.wait_ready_to_arm()

        # fly with the healthy lateral baseline from copter-gps-for-yaw.parm
        # so the EKF fuses GPS yaw and learns that the compass agrees with
        # it, which arms the fallback
        self.takeoff(10, mode='GUIDED')
        self.delay_sim_time(10, reason="learning that the compass agrees with GPS yaw")

        # reconfigure the antennas to a vertical baseline in flight.  The
        # driver keeps publishing yaw: the simulated antennas keep 0.3 m of
        # horizontal separation, and the baseline length and vertical drop
        # still match the configured offsets, so every driver-side check
        # passes.  The configured offset the yaw is calculated from is
        # purely vertical, so the EKF rejects every measurement
        self.context_collect('STATUSTEXT')
        self.set_parameters({
            "GPS1_POS_X": 0.0, "GPS1_POS_Y": 0.0, "GPS1_POS_Z": -0.45,
            "GPS2_POS_X": 0.0, "GPS2_POS_Y": 0.0, "GPS2_POS_Z": 0.45,
            "SIM_GPS1_POS_X": 0.0, "SIM_GPS1_POS_Y": -0.15, "SIM_GPS1_POS_Z": -0.44,
            "SIM_GPS2_POS_X": 0.0, "SIM_GPS2_POS_Y": 0.15, "SIM_GPS2_POS_Z": 0.44,
        })

        # the fallback requires 10 seconds with no usable GPS yaw
        self.wait_statustext("yaw fallback active", timeout=60, check_context=True)

        # the compass must now be steering the EKF yaw: confirm it tracks truth
        self.delay_sim_time(5, reason="letting compass fallback settle")
        att = self.assert_receive_message("ATTITUDE")
        sim = self.assert_receive_message("SIMSTATE")
        yaw_err_deg = abs(mavextra.wrap_180(math.degrees(att.yaw) - math.degrees(sim.yaw)))
        if yaw_err_deg > 20:
            raise NotAchievedException(
                "Yaw not tracking truth under compass fallback (err=%.1f deg)" % yaw_err_deg)

        self.do_RTL()
