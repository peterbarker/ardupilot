'''
Copter autotest: FlyRangeFinderMAVlink

AP_FLAKE8_CLEAN
'''

from pymavlink import mavutil

from vehicle_test_suite import NotAchievedException

from test_discovery import TestGroup
from test_discovery import testcase


class FlyRangeFinderMAVlink(TestGroup):
    def fly_rangefinder_mavlink_distance_sensor(self):
        self.start_subtest("Test mavlink rangefinder using DISTANCE_SENSOR messages")
        self.context_push()
        self.set_parameters({
            "RTL_ALT_TYPE": 0,
            "LGR_ENABLE": 1,
            "LGR_DEPLOY_ALT": 1,
            "LGR_RETRACT_ALT": 10, # metres
            "SERVO10_FUNCTION": 29
        })
        ex = None
        try:
            self.set_parameter("SERIAL5_PROTOCOL", 1)
            self.set_parameter("RNGFND1_TYPE", 10)
            self.reboot_sitl()
            self.set_parameter("RNGFND1_MAX", 327.67)

            self.progress("Should be unhealthy while we don't send messages")
            self.assert_sensor_state(mavutil.mavlink.MAV_SYS_STATUS_SENSOR_LASER_POSITION, True, True, False)

            # we are interacting with the autopilot, reduce chance of
            # hitting timeouts on supplied data:
            self.context_set_speedup(1)
            self.progress("Should be healthy while we're sending good messages")
            tstart = self.get_sim_time()
            while True:
                if self.get_sim_time() - tstart > 5:
                    raise NotAchievedException("Sensor did not come good")
                self.mav.mav.distance_sensor_send(
                    0,  # time_boot_ms
                    10, # min_distance
                    50, # max_distance
                    20, # current_distance
                    mavutil.mavlink.MAV_DISTANCE_SENSOR_LASER, # type
                    21, # id
                    mavutil.mavlink.MAV_SENSOR_ROTATION_PITCH_270, # orientation
                    255 # covariance
                )
                if self.sensor_has_state(mavutil.mavlink.MAV_SYS_STATUS_SENSOR_LASER_POSITION, True, True, True):
                    self.progress("Sensor has good state")
                    break
                self.delay_sim_time(0.1, reason="poll interval")

            self.progress("Should be unhealthy again if we stop sending messages")
            self.wait_sensor_state(mavutil.mavlink.MAV_SYS_STATUS_SENSOR_LASER_POSITION, True, True, False)

            self.progress("Landing gear should deploy with current_distance below min_distance")
            self.change_mode('STABILIZE')
            timeout = 60
            tstart = self.get_sim_time()
            while not self.armed():
                if self.get_sim_time() - tstart > timeout:
                    raise NotAchievedException("Failed to become armable after %f seconds" % timeout)
                self.mav.mav.distance_sensor_send(
                    0,  # time_boot_ms
                    100, # min_distance (cm)
                    2500, # max_distance (cm)
                    200, # current_distance (cm)
                    mavutil.mavlink.MAV_DISTANCE_SENSOR_LASER, # type
                    21, # id
                    mavutil.mavlink.MAV_SENSOR_ROTATION_PITCH_270, # orientation
                    255  # covariance
                )
                try:
                    self.arm_vehicle()
                except Exception:  # noqa: BLE001
                    pass
            self.delay_sim_time(1, reason="servo function maps to update")  # servo function maps only periodically updated
#            self.send_debug_trap()

            self.run_cmd(
                mavutil.mavlink.MAV_CMD_AIRFRAME_CONFIGURATION,
                p2=0,  # deploy
            )

            self.context_collect("STATUSTEXT")
            tstart = self.get_sim_time()
            while True:
                if self.get_sim_time_cached() - tstart > 5:
                    raise NotAchievedException("Retraction did not happen")
                self.mav.mav.distance_sensor_send(
                    0,  # time_boot_ms
                    100, # min_distance (cm)
                    6000, # max_distance (cm)
                    1500, # current_distance (cm)
                    mavutil.mavlink.MAV_DISTANCE_SENSOR_LASER, # type
                    21, # id
                    mavutil.mavlink.MAV_SENSOR_ROTATION_PITCH_270, # orientation
                    255  # covariance
                )
                self.delay_sim_time(0.1, reason="poll interval")
                try:
                    self.wait_text("LandingGear: RETRACT", check_context=True, timeout=0.1)
                except Exception:  # noqa: BLE001
                    continue
                self.progress("Retracted")
                break
#            self.send_debug_trap()
            while True:
                if self.get_sim_time_cached() - tstart > 5:
                    raise NotAchievedException("Deployment did not happen")
                self.progress("Sending distance-sensor message")
                self.mav.mav.distance_sensor_send(
                    0, # time_boot_ms
                    300, # min_distance
                    500, # max_distance
                    250, # current_distance
                    mavutil.mavlink.MAV_DISTANCE_SENSOR_LASER, # type
                    21, # id
                    mavutil.mavlink.MAV_SENSOR_ROTATION_PITCH_270, # orientation
                    255 # covariance
                )
                try:
                    self.wait_text("LandingGear: DEPLOY", check_context=True, timeout=0.1)
                except Exception:  # noqa: BLE001
                    continue
                self.progress("Deployed")
                break
            self.disarm_vehicle()

        except Exception as e:  # noqa: BLE001
            self.print_exception_caught(e)
            ex = e
        self.context_pop()
        self.reboot_sitl()
        if ex is not None:
            raise ex

    @testcase(bucket="CopterTests1a")
    def FlyRangeFinderMAVlink(self):
        '''fly mavlink-connected rangefinder'''
        self.fly_rangefinder_mavlink_distance_sensor()

        # explicit test for the mavlink driver as it doesn't play so nice:
        self.set_parameters({
            "SERIAL5_PROTOCOL": 1,
            "RNGFND1_TYPE": 10,
            "RTL_ALT_M": 5,
            "RTL_ALT_TYPE": 1,
            "SIM_TERRAIN": 0,
        })
        self.customise_SITL_commandline(['--serial5=sim:rf_mavlink'])

        self.change_mode('GUIDED')
        self.wait_ready_to_arm()
        self.arm_vehicle()
        expected_alt = 5
        self.user_takeoff(alt_min=expected_alt)

        self.context_push()
        self.context_set_message_rate_hz('RANGEFINDER', self.sitl_streamrate())

        tstart = self.get_sim_time()
        while True:
            if self.get_sim_time() - tstart > 5:
                raise NotAchievedException("Mavlink rangefinder not working")
            rf = self.assert_receive_message("RANGEFINDER")
            gpi = self.assert_receive_message('GLOBAL_POSITION_INT')
            if abs(rf.distance - gpi.relative_alt/1000.0) > 1:
                self.progress("rangefinder alt (%s) disagrees with global-position-int.relative_alt (%s)" %
                              (rf.distance, gpi.relative_alt/1000.0))
                continue

            ds = self.assert_receive_message("DISTANCE_SENSOR", timeout=2, verbose=True)
            if abs(ds.current_distance/100.0 - gpi.relative_alt/1000.0) > 1:
                self.progress(
                    "distance sensor.current_distance (%f) disagrees with global-position-int.relative_alt (%s)" %
                    (ds.current_distance/100.0, gpi.relative_alt/1000.0))
                continue
            break

        self.context_pop()

        self.progress("mavlink rangefinder OK")
        self.land_and_disarm()
