'''
QuadPlane autotest: WindEstimateConsistency

AP_FLAKE8_CLEAN
'''

from vehicle_test_suite import NotAchievedException

from test_discovery import TestGroup
from test_discovery import testcase


class WindEstimateConsistency(TestGroup):
    @testcase(bucket="QuadPlaneTests1c")
    def WindEstimateConsistency(self):
        '''test that DCM and EKF3 roughly agree on wind speed and direction'''
        self.set_parameters({
            'SIM_WIND_SPD': 10,   # metres/second
            'SIM_WIND_DIR': 315,  # from the North-West
        })
        self.change_mode('TAKEOFF')
        self.wait_ready_to_arm()
        self.arm_vehicle()
        self.delay_sim_time(180, reason="wind estimate data collection")
        mlog = self.dfreader_for_current_onboard_log()
        self.fly_home_land_and_disarm()

        self.progress("Inspecting dataflash log")
        match_start_time = None
        dcm = None
        xkf2 = None
        while True:
            m = mlog.recv_match(
                type=['DCM', 'XKF2'],
                blocking=True,
            )
            if m is None:
                raise NotAchievedException("Did not see wind estimates match")

            m_type = m.get_type()
            if m_type == 'DCM':
                dcm = m
            else:
                xkf2 = m
            if dcm is None or xkf2 is None:
                continue

            now = m.TimeUS * 1e-6

            matches_east = abs(dcm.VWE-xkf2.VWE) < 1.5
            matches_north = abs(dcm.VWN-xkf2.VWN) < 1.5

            matches = matches_east and matches_north

            if not matches:
                match_start_time = None
                continue

            if match_start_time is None:
                match_start_time = now
                continue

            if now - match_start_time > 60:
                self.progress("Wind estimates correlated")
                break
