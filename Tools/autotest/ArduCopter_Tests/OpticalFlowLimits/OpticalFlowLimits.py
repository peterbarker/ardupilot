'''
Copter autotest: OpticalFlowLimits

AP_FLAKE8_CLEAN
'''

import math

from vehicle_test_suite import NotAchievedException

from test_discovery import TestGroup
from test_discovery import testcase


class OpticalFlowLimits(TestGroup):
    @testcase(bucket="CopterTests1e")
    def OpticalFlowLimits(self):
        '''test EKF navigation limiting'''
        self.set_parameters({
            "SIM_FLOW_ENABLE": 1,
            "FLOW_TYPE": 10,
            "SIM_GPS1_ENABLE": 0,
            "SIM_TERRAIN": 0,
        })

        self.configure_EKFs_to_use_optical_flow_instead_of_GPS()

        self.set_analog_rangefinder_parameters()

        self.reboot_sitl()

        # we can't takeoff in loiter as we need flow healthy
        self.takeoff(altitude_min=5, mode='ALT_HOLD', require_absolute=False, takeoff_throttle=1800)
        self.change_mode('LOITER')

        # speed should be limited to <10m/s
        self.set_rc(2, 1000)

        tstart = self.get_sim_time()
        timeout = 60
        started_climb = False
        while self.get_sim_time_cached() - tstart < timeout:
            m = self.assert_receive_message('GLOBAL_POSITION_INT')
            spd = math.sqrt(m.vx**2 + m.vy**2) * 0.01
            alt = m.relative_alt*0.001

            # calculate max speed from altitude above the ground
            margin = 2.0
            max_speed = alt * 1.5 + margin
            self.progress("%0.1f: Low Speed: %f (want <= %u) alt=%.1f" %
                          (self.get_sim_time_cached() - tstart,
                           spd,
                           max_speed, alt))
            if spd > max_speed:
                raise NotAchievedException(("Speed should be limited by"
                                            "EKF optical flow limits"))

            # after 30 seconds start climbing
            if not started_climb and self.get_sim_time_cached() - tstart > 30:
                started_climb = True
                self.set_rc(3, 1900)
                self.progress("Moving higher")

            # check altitude is not climbing above 35m
            if alt > 35:
                raise NotAchievedException("Alt should be limited by EKF optical flow limits")
        self.reboot_sitl(force=True)
