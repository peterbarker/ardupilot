'''
Copter autotest: GPSBlending

AP_FLAKE8_CLEAN
'''

from vehicle_test_suite import NotAchievedException

from test_discovery import TestGroup
from test_discovery import testcase


class GPSBlending(TestGroup):
    @testcase(bucket="CopterTests1e")
    def GPSBlending(self):
        '''Test GPS Blending'''
        '''ensure we get dataflash log messages for blended instance'''

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
        passes = 0
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
            measurements[m.I] = (m.Lat, m.Lng)
            if len(measurements) == 3:
                # check lat:
                for n in 0, 1:
                    expected_blended = (measurements[0][n] + measurements[1][n])/2
                    epsilon = 0.0000002
                    error = abs(measurements[2][n] - expected_blended)
                    if error > epsilon:
                        raise NotAchievedException("Blended diverged")
                current_ts = None
                passes += 1

        if passes == 0:
            raise NotAchievedException("Did not see three GPS measurements!")
