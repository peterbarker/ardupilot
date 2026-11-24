'''
Copter autotest: GPSGlitchVelocity

AP_FLAKE8_CLEAN
'''

import math

from vehicle_test_suite import NotAchievedException

from test_discovery import TestGroup
from test_discovery import testcase


class GPSGlitchVelocity(TestGroup):
    @testcase(bucket="CopterTests2c")
    def GPSGlitchVelocity(self):
        '''check SIM_GPS1_GLTV offsets the GPS velocity and moves the reported position with it'''
        self.wait_ready_to_arm()
        # the vehicle sits disarmed, so SIMSTATE stays the true position throughout
        truth = self.get_location('SIMSTATE')

        def gps_state():
            '''GPS_RAW_INT position relative to truth (metres north, east), speed (m/s), course (deg), sim time'''
            m = self.assert_receive_message('GPS_RAW_INT')
            north = math.radians(m.lat * 1e-7 - truth.lat) * 6378137
            east = math.radians(m.lon * 1e-7 - truth.lng) * 6378137 * math.cos(math.radians(truth.lat))
            return north, east, m.vel * 0.01, m.cog * 0.01, self.get_sim_time_cached()

        self.progress("Applying GPS velocity glitch of 1m/s north, 2m/s east")
        self.set_parameters({
            "SIM_GPS1_GLTV_X": 1,
            "SIM_GPS1_GLTV_Y": 2,
        })
        self.delay_sim_time(2, reason="let the glitch reach the GPS output")
        (n0, e0, speed, course, t0) = gps_state()
        self.progress(f"GPS speed={speed:.2f}m/s course={course:.1f}deg")
        if abs(speed - math.sqrt(5)) > 0.3:
            raise NotAchievedException(f"GPS speed {speed:.2f}m/s, expected {math.sqrt(5):.2f}m/s")
        if abs(course - math.degrees(math.atan2(2, 1))) > 5:
            raise NotAchievedException(f"GPS course {course:.1f}deg, expected {math.degrees(math.atan2(2, 1)):.1f}deg")

        # the reported position must move consistently with the offset velocity
        self.delay_sim_time(10, reason="let the position drift")
        (n1, e1, _, _, t1) = gps_state()
        dt = t1 - t0
        self.progress(f"GPS moved north={n1 - n0:.2f}m east={e1 - e0:.2f}m in {dt:.1f}s")
        if abs((n1 - n0) - 1 * dt) > 1.5 or abs((e1 - e0) - 2 * dt) > 1.5:
            raise NotAchievedException("GPS position did not drift with the velocity glitch")

        # zeroing every component clears the accumulated position offset
        self.progress("Removing GPS velocity glitch")
        self.set_parameters({
            "SIM_GPS1_GLTV_X": 0,
            "SIM_GPS1_GLTV_Y": 0,
        })
        self.delay_sim_time(2, reason="let the cleared glitch reach the GPS output")
        (n2, e2, speed, _, _) = gps_state()
        self.progress(f"GPS offset from truth={math.hypot(n2, e2):.2f}m speed={speed:.2f}m/s")
        if math.hypot(n2, e2) > 1.5 or speed > 0.3:
            raise NotAchievedException("GPS velocity glitch offset did not clear")
