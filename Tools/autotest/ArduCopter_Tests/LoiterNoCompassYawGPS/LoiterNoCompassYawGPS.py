'''
Copter autotest: LoiterNoCompassYawGPS

AP_FLAKE8_CLEAN
'''

from vehicle_test_suite import NotAchievedException

from test_discovery import TestGroup
from test_discovery import testcase


class LoiterNoCompassYawGPS(TestGroup):
    @testcase(bucket="CopterTests2a")
    def LoiterNoCompassYawGPS(self):
        '''Loiter with GPS and no optical flow, compass not an EK3 yaw source'''
        # GPS-aided, no optical flow. Compass enabled but not an EK3 yaw source
        # (EK3_SRC1_YAW=0/None), so yaw comes from the GSF estimator, which only
        # aligns once the vehicle has GPS velocity to work with. Loiter cannot
        # be entered on the ground (no heading -> no absolute position), so
        # exercise the supported path: arm and climb in ALT_HOLD, fly forward
        # to let GSF align yaw, then switch to Loiter and confirm it holds.
        self.set_parameters({
            "AHRS_EKF_TYPE": 3,
            "EK3_ENABLE": 1,
            "EK2_ENABLE": 0,
            "EK3_SRC1_YAW": 0,  # None - compass enabled but not used for yaw
            "SIM_FLOW_ENABLE": 0,
            "FLOW_TYPE": 0,
        })
        self.reboot_sitl()

        # GSF cannot align yaw while stationary, so there is no absolute
        # position yet; arm and climb in ALT_HOLD, which needs no position.
        self.wait_gps_fix_type_gte(3)
        self.context_collect("STATUSTEXT")
        self.change_mode('ALT_HOLD')
        self.wait_prearm_sys_status_healthy(timeout=120)
        self.arm_vehicle()
        self.set_rc(3, 1700)
        self.wait_altitude(8, 20, relative=True, timeout=60)
        self.set_rc(3, 1500)

        # fly forward so the GSF has a velocity to align yaw from
        self.set_rc(2, 1300)
        self.wait_statustext("yaw aligned", check_context=True, timeout=60)
        self.set_rc(2, 1500)

        # yaw is aligned, so the EKF now has an absolute position and Loiter is
        # usable; switch to it and confirm it stays engaged and armed.
        self.wait_ekf_happy(require_absolute=True, timeout=30)
        self.change_mode('LOITER')
        self.delay_sim_time(15, "confirm Loiter stays engaged and armed")
        self.wait_mode('LOITER')
        if not self.armed():
            raise NotAchievedException("Disarmed during Loiter without compass yaw source")

        self.do_RTL()
