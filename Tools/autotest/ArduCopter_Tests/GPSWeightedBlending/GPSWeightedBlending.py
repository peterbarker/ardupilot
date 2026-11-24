'''
Copter autotest: GPSWeightedBlending

AP_FLAKE8_CLEAN
'''

from vehicle_test_suite import NotAchievedException

from test_discovery import TestGroup
from test_discovery import testcase


class GPSWeightedBlending(TestGroup):
    @testcase(bucket="CopterTests1e")
    def GPSWeightedBlending(self):
        '''Test GPS Weighted Blending'''

        self.context_push()

        # configure:
        self.set_parameters({
            "WP_YAW_BEHAVIOR": 0,  # do not yaw
            "GPS2_TYPE": 1,
            "SIM_GPS2_TYPE": 1,
            "SIM_GPS2_ENABLE": 1,
            "SIM_GPS1_POS_X": 1.0,
            "SIM_GPS1_POS_Y": -1.0,
            "SIM_GPS2_POS_X": -1.0,
            "SIM_GPS2_POS_Y": 1.0,
            "GPS_AUTO_SWITCH": 2,
        })
        # configure velocity errors such that the 1st GPS should be
        # 4/5, second GPS 1/5 of result (0.5**2)/((0.5**2)+(1.0**2))
        self.set_parameters({
            "SIM_GPS1_VERR_X": 0.3,  # m/s
            "SIM_GPS1_VERR_Y": 0.4,
            "SIM_GPS2_VERR_X": 0.6,  # m/s
            "SIM_GPS2_VERR_Y": 0.8,
            "GPS_BLEND_MASK": 4,  # use only speed for blend calculations
        })
        self.reboot_sitl()

        alt = 10
        self.takeoff(alt, mode='GUIDED')
        self.fly_guided_move_local(30, 0, alt)
        self.fly_guided_move_local(30, 30, alt)
        self.fly_guided_move_local(0, 30, alt)
        self.fly_guided_move_local(0, 0, alt)
        self.change_mode('LAND')

        current_log_file = self.dfreader_for_current_onboard_log()

        self.wait_disarmed()

        # ensure that the blended solution is always about half-way
        # between the two GPSs:
        current_ts = None
        while True:
            m = current_log_file.recv_match(type='GPS')
            if m is None:
                break
            if current_ts is None:
                if m.I != 0:  # noqa
                    continue
                current_ts = m.TimeUS
                measurements = {}
            if m.TimeUS != current_ts:
                current_ts = None
                continue
            measurements[m.I] = (m.Lat, m.Lng, m.Alt)
            if len(measurements) == 3:
                # check lat:
                for n in 0, 1, 2:
                    expected_blended = 0.8*measurements[0][n] + 0.2*measurements[1][n]
                    axis_epsilons = [0.0000002, 0.0000002, 0.2]
                    epsilon = axis_epsilons[n]
                    error = abs(measurements[2][n] - expected_blended)
                    if error > epsilon:
                        raise NotAchievedException(f"Blended diverged {n=} {measurements[0][n]=} {measurements[1][n]=}")
                current_ts = None

        self.context_pop()
        self.reboot_sitl()
