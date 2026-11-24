'''
QuadPlane autotest: TakeoffCheck

AP_FLAKE8_CLEAN
'''

from pymavlink import mavutil

from vehicle_test_suite import NotAchievedException

from test_discovery import TestGroup
from test_discovery import testcase


class TakeoffCheck(TestGroup):
    @testcase(bucket="QuadPlaneTests1c")
    def TakeoffCheck(self):
        '''Test takeoff check - auto mode'''
        self.set_parameters({
            "AHRS_EKF_TYPE": 10,
            'SIM_ESC_TELEM': 1,
        })

        self.start_subtest("Test blocking doesn't occur with in-range RPM")
        self.context_push()
        self.context_collect('STATUSTEXT')
        self.set_parameters({
            'SIM_VIB_MOT_MAX': 150, # Hz, 9000 RPM, ensures the test fails if check occurs after takeoff starts
            'SIM_ESC_ARM_RPM': 1000,
            'Q_TKOFF_RPM_MIN': 900,
            'Q_TKOFF_RPM_MAX': 1100,
        })
        self.upload_simple_relhome_mission([
            (mavutil.mavlink.MAV_CMD_NAV_VTOL_TAKEOFF, 0, 0, 1),
            (mavutil.mavlink.MAV_CMD_NAV_VTOL_LAND, 0, 0, 0),
        ])
        self.change_mode('AUTO')
        self.wait_ready_to_arm()
        self.arm_vehicle()
        self.wait_current_waypoint(2)
        self.wait_disarmed()
        self.set_current_waypoint(0, check_afterwards=False)
        # ensure no spurious takeoff blocked warnings during spoolup
        for m in self.context_collection('STATUSTEXT'):
            if "Takeoff blocked" in m.text:
                raise NotAchievedException("Spurious takeoff blocked message: %s" % m.text)
        self.context_pop()

        self.start_subtest("Ensure blocked if motors don't spool up")
        self.context_push()
        self.set_parameters({
            'SIM_ESC_ARM_RPM': 500,
            'Q_TKOFF_RPM_MIN': 1000,
        })
        self.upload_simple_relhome_mission([
            (mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 30),
            (mavutil.mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH, 0, 0, 0),
        ])
        self.test_takeoff_check_mode("AUTO", force_disarm=True)
        self.context_pop()

        self.start_subtest("Ensure blocked if virtual motors are missing virtual props")
        self.context_push()
        self.set_parameters({
            'Q_TKOFF_RPM_MIN': 1,
            'Q_TKOFF_RPM_MAX': 3,
        })
        self.test_takeoff_check_mode("AUTO", force_disarm=True)
        self.context_pop()
