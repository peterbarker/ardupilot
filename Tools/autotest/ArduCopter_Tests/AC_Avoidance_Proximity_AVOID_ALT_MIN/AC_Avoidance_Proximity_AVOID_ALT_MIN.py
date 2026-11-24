'''
Copter autotest: AC_Avoidance_Proximity_AVOID_ALT_MIN

AP_FLAKE8_CLEAN
'''

from pymavlink import mavutil
from pymavlink.rotmat import Vector3

from vehicle_test_suite import AutoTestTimeoutException
from vehicle_test_suite import NotAchievedException

from test_discovery import TestGroup
from test_discovery import testcase


class AC_Avoidance_Proximity_AVOID_ALT_MIN(TestGroup):
    @testcase(bucket="CopterTests1d")
    def AC_Avoidance_Proximity_AVOID_ALT_MIN(self):
        '''Test proximity avoidance with AVOID_ALT_MIN'''
        self.set_parameters({
            "PRX1_TYPE": 2,
            "AVOID_ALT_MIN": 10,
        })
        self.set_analog_rangefinder_parameters()
        self.reboot_sitl()

        self.change_mode('LOITER')
        self.wait_ekf_happy()

        self.context_set_speedup(20)

        tstart = self.get_sim_time()
        while True:
            if self.armed(cached=True):
                break
            if self.get_sim_time_cached() - tstart > 60:
                raise AutoTestTimeoutException("Did not arm")
            self.mav.mav.distance_sensor_send(
                0, # time_boot_ms
                10, # min_distance cm
                500, # max_distance cm
                400, # current_distance cm
                mavutil.mavlink.MAV_DISTANCE_SENSOR_LASER, # type
                26, # id
                mavutil.mavlink.MAV_SENSOR_ROTATION_NONE, # orientation
                255  # covariance
            )
            self.send_mavlink_arm_command()
            # make a minimal effort to not do ridiculous amounts of
            # mavlink to the autopilot:
            self.do_timesync_roundtrip()

        self.takeoff(15, mode='LOITER')
        self.progress("Poking vehicle; should avoid")

        def shove(a, b):
            self.mav.mav.distance_sensor_send(
                0,  # time_boot_ms
                10, # min_distance cm
                500, # max_distance cm
                20, # current_distance cm
                mavutil.mavlink.MAV_DISTANCE_SENSOR_LASER, # type
                21, # id
                mavutil.mavlink.MAV_SENSOR_ROTATION_NONE, # orientation
                255  # covariance
            )
        self.wait_speed_vector_bf(
            Vector3(-0.4, 0.0, 0.0),
            timeout=10,
            called_function=shove,
        )

        self.change_alt(5)

        tstart = self.get_sim_time()
        while True:
            if self.get_sim_time_cached() - tstart > 10:
                break
            vel = self.get_body_frame_velocity()
            if vel.length() > 0.5:
                raise NotAchievedException("Moved too much (%s)" %
                                           (str(vel),))
            shove(None, None)

        self.disarm_vehicle(force=True)
