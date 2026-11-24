'''
Copter autotest: GPSForYawWindEstimation

AP_FLAKE8_CLEAN
'''

from pymavlink import mavextra

from vehicle_test_suite import NotAchievedException

from test_discovery import TestGroup
from test_discovery import testcase


class GPSForYawWindEstimation(TestGroup):
    @testcase(bucket="CopterTests1d")
    def GPSForYawWindEstimation(self):
        '''Test drag-based wind estimation when using GPS yaw'''
        wind_speed = 5
        wind_direction = 45
        self.load_default_params_file("copter-gps-for-yaw.parm")
        self.set_parameters({
            "EK3_DRAG_BCOEF_X": 9.5,
            "EK3_DRAG_BCOEF_Y": 9.5,
            "EK3_DRAG_MCOEF": 0.082,
            "SIM_WIND_DIR": wind_direction,
            "SIM_WIND_SPD": wind_speed,
            "SIM_WIND_T": 1,
        })

        for yaw_source in (2, 3):
            self.start_subtest("EK3_SRC1_YAW=%u" % yaw_source)
            self.set_parameter("EK3_SRC1_YAW", yaw_source)
            self.reboot_sitl()

            self.wait_gps_fix_type_gte(6, message_type="GPS2_RAW", verbose=True)
            m = self.assert_receive_message("GPS2_RAW")
            if abs(m.yaw - 27000) > 500:
                raise NotAchievedException(
                    "Expected GPS yaw near 270deg with EK3_SRC1_YAW=%u, got %f" %
                    (yaw_source, m.yaw * 0.01))
            self.wait_ready_to_arm()
            self.takeoff(10, mode="LOITER")

            # Rotate to provide drag observations in both body axes.
            try:
                self.set_rc(4, 1400)
                tstart = self.get_sim_time()
                last_report = 0
                while True:
                    if self.get_sim_time_cached() - tstart > 60:
                        raise NotAchievedException(
                            "Wind estimate did not converge with EK3_SRC1_YAW=%u" % yaw_source)
                    m = self.assert_receive_message("WIND")
                    speed_error = abs(m.speed - wind_speed)
                    direction_error = abs(mavextra.wrap_180(m.direction - wind_direction))
                    if self.get_sim_time_cached() - last_report > 5:
                        self.progress(
                            "EK3_SRC1_YAW=%u wind speed=%f direction=%f" %
                            (yaw_source, m.speed, m.direction))
                        last_report = self.get_sim_time_cached()
                    if speed_error < 1 and direction_error < 15:
                        break
            finally:
                self.set_rc(4, 1500)
                self.land_and_disarm()
