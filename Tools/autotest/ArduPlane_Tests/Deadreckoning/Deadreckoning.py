'''
Plane autotest: Deadreckoning

AP_FLAKE8_CLEAN
'''

from pymavlink import mavutil

import vehicle_test_suite

from vehicle_test_suite import AltFrame
from vehicle_test_suite import NotAchievedException

from test_discovery import TestGroup
from test_discovery import testcase


class Deadreckoning(TestGroup):
    def deadreckoning_main(self, disable_airspeed_sensor=False):
        self.context_push()
        self.set_parameter("EK3_OPTIONS", 1)
        self.set_parameter("AHRS_OPTIONS", 3)
        self.set_parameter("LOG_REPLAY", 1)
        self.reboot_sitl()
        self.wait_ready_to_arm()

        if disable_airspeed_sensor:
            max_allowed_divergence = 300
        else:
            max_allowed_divergence = 150
        # sample the published position as fast as the link allows; a
        # position glitch can last a single AHRS cycle, which the default
        # 10Hz GLOBAL_POSITION_INT rate usually steps over.  0.8*loop rate
        # is the fastest a message may be streamed:
        fast_rate_hz = int(0.8 * self.get_parameter("SCHED_LOOP_RATE"))
        self.set_message_rate_hz('GLOBAL_POSITION_INT', fast_rate_hz)
        self.set_message_rate_hz('SIMSTATE', fast_rate_hz)
        self.install_message_hook_context(vehicle_test_suite.TestSuite.ValidateGlobalPositionIntAgainstSimState(self, max_allowed_divergence=max_allowed_divergence))  # noqa

        try:
            # wind is from the West:
            self.set_parameter("SIM_WIND_DIR", 270)
            # light winds:
            self.set_parameter("SIM_WIND_SPD", 10)
            if disable_airspeed_sensor:
                self.set_parameter("ARSPD_USE", 0)

            target_alt = 100
            loc = self.offset_location_ne(self.home_position_as_location(), 500, 500)
            loc.set_alt_m(target_alt, AltFrame.ABOVE_HOME)
            self.takeoff(50)
            self.run_cmd_int(
                mavutil.mavlink.MAV_CMD_DO_REPOSITION,
                p1=0,
                p2=mavutil.mavlink.MAV_DO_REPOSITION_FLAGS_CHANGE_MODE,
                p5=int(loc.lat * 1e7),
                p6=int(loc.lng * 1e7),
                p7=target_alt,    # alt
                frame=mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
            )
            self.wait_location(loc, accuracy=100, height_accuracy=10)
            self.progress("Orbit with GPS and learn wind")
            # allow longer to learn wind if there is no airspeed sensor
            if disable_airspeed_sensor:
                self.delay_sim_time(180, reason="wind learning without airspeed sensor")
            else:
                self.delay_sim_time(20, reason="wind learning in orbit")
            self.set_parameter("SIM_GPS1_ENABLE", 0)
            self.progress("Continue orbit without GPS")
            self.delay_sim_time(20, reason="dead reckoning data collection")
            self.change_mode("RTL")
            self.wait_distance_to_home(100, 200, timeout=200)
            # go into LOITER to create additional time for a GPS re-enable test
            self.change_mode("LOITER")
            self.set_parameter("SIM_GPS1_ENABLE", 1)
            t_enabled = self.get_sim_time()
            # The EKF should wait for GPS checks to pass when we are still able to navigate using dead reckoning
            # to prevent bad GPS being used when coming back after loss of lock due to interence.
            self.wait_ekf_flags(mavutil.mavlink.ESTIMATOR_POS_HORIZ_ABS, 0, timeout=15)
            if self.get_sim_time() < (t_enabled+9):
                raise NotAchievedException("GPS use re-started too quickly")
            # wait for EKF and vehicle position to stabilise, then test response to jamming
            self.delay_sim_time(20, reason="EKF and position to stabilise")

            self.set_parameter("AHRS_OPTIONS", 1)
            self.set_parameter("SIM_GPS1_JAM", 1)
            self.delay_sim_time(13, reason="GPS jamming simulation")
            self.set_parameter("SIM_GPS1_JAM", 0)
            t_enabled = self.get_sim_time()
            # The EKF should wait for GPS checks to pass when we are still able to navigate using dead reckoning
            # to prevent bad GPS being used when coming back after loss of lock due to interence.
            # The EKF_STATUS_REPORT does not tell us when the good to align check passes, so the minimum time
            # value of 3.0 seconds is an arbitrary value set on inspection of dataflash logs from this test
            self.wait_ekf_flags(mavutil.mavlink.ESTIMATOR_POS_HORIZ_ABS, 0, timeout=20)
            time_since_jamming_stopped = self.get_sim_time() - t_enabled
            if time_since_jamming_stopped < 3:
                raise NotAchievedException("GPS use re-started %f sec after jamming stopped" % time_since_jamming_stopped)
            self.set_rc(3, 1000)
            self.fly_home_land_and_disarm()
        finally:
            pass

        self.context_pop()
        self.reboot_sitl()

    @testcase(bucket="PlaneTests1b")
    def Deadreckoning(self):
        '''Test deadreckoning support'''
        self.deadreckoning_main()

    @testcase(bucket="PlaneTests1a")
    def DeadreckoningNoAirSpeed(self):
        '''Test deadreckoning support with no airspeed sensor'''
        self.deadreckoning_main(disable_airspeed_sensor=True)
