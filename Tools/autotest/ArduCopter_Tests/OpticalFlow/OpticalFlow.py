'''
Copter autotest: OpticalFlow

AP_FLAKE8_CLEAN
'''

import math

import vehicle_test_suite

from vehicle_test_suite import NotAchievedException

from test_discovery import TestGroup
from test_discovery import testcase


class OpticalFlow(TestGroup):
    @testcase(bucket="CopterTests1e")
    def OpticalFlow(self):
        '''test OpticalFlow in flight'''
        self.start_subtest("Make sure no crash if no rangefinder")

        self.set_parameters({
            "SIM_FLOW_ENABLE": 1,
            "SIM_FLOW_RND": 0.02,
            "FLOW_TYPE": 10,
        })

        self.set_analog_rangefinder_parameters()

        self.reboot_sitl()

        self.change_mode('LOITER')

        class CheckOpticalFlow(vehicle_test_suite.TestSuite.MessageHook):
            '''ensures OPTICAL_FLOW data matches other data'''
            def __init__(self, suite):
                super().__init__(suite)

                self.flow_rate_rads = 0
                self.rangefinder_distance = 0
                self.gps_speed = 0
                self.last_debug_time = 0
                self.distance_sensor_distance = 0

            def progress_prefix(self):
                return "COF: "

            def process(self, mav, m):
                m_type = m.get_type()
                if m_type == "OPTICAL_FLOW":
                    self.flow_rate_rads = math.sqrt(m.flow_comp_m_x**2+m.flow_comp_m_y**2)
                elif m_type == "RANGEFINDER":
                    self.rangefinder_distance = m.distance
                elif m_type == "DISTANCE_SENSOR":
                    self.distance_sensor_distance = m.current_distance * 0.01
                elif m_type == "GPS_RAW_INT":
                    self.gps_speed = m.vel/100.0  # cm/s -> m/s

                of_speed = self.flow_rate_rads * self.rangefinder_distance
                if abs(of_speed - self.gps_speed) > 3:
                    raise NotAchievedException(f"gps={self.gps_speed} vs of={of_speed} mismatch")
                of_speed = self.flow_rate_rads * self.distance_sensor_distance
                if abs(of_speed - self.gps_speed) > 3:
                    raise NotAchievedException(f"gps={self.gps_speed} vs of={of_speed} mismatch")

                now = self.suite.get_sim_time_cached()
                if now - self.last_debug_time > 5:
                    self.last_debug_time = now
                    self.progress(f"gps={self.gps_speed} of={of_speed}")

        self.context_set_message_rate_hz('RANGEFINDER', self.sitl_streamrate())

        check_optical_flow = CheckOpticalFlow(self)
        self.install_message_hook_context(check_optical_flow)

        self.fly_generic_mission("CMAC-copter-navtest.txt")
