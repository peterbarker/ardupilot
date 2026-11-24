'''
QuadPlane autotest: ICEngine

AP_FLAKE8_CLEAN
'''

from vehicle_test_suite import AutoTestTimeoutException
from vehicle_test_suite import NotAchievedException

from test_discovery import TestGroup
from test_discovery import testcase


class ICEngine(TestGroup):
    @testcase(bucket="QuadPlaneTests1a")
    def ICEngine(self):
        '''Test ICE Engine support'''
        rc_engine_start_chan = 11
        self.setup_ICEngine_vehicle()

        self.wait_ready_to_arm()
        self.wait_rpm(1, 0, 0, minimum_duration=1)
        self.arm_vehicle()
        self.wait_rpm(1, 0, 0, minimum_duration=1)
        self.context_collect("STATUSTEXT")
        self.progress("Setting engine-start RC switch to HIGH")
        self.set_rc(rc_engine_start_chan, 2000)
        self.wait_statustext("Starting engine", check_context=True)
        self.wait_rpm(1, 300, 400, minimum_duration=1)
        self.progress("Setting engine-start RC switch to MID")
        self.set_rc(rc_engine_start_chan, 1500)
        self.progress("Setting full throttle")
        self.set_rc(3, 2000)
        self.wait_rpm(1, 6500, 7500, minimum_duration=30, timeout=40)
        self.progress("Setting min-throttle")
        self.set_rc(3, 1000)
        self.wait_rpm(1, 65, 75, minimum_duration=1)
        self.progress("Setting engine-start RC switch to LOW")
        self.set_rc(rc_engine_start_chan, 1000)
        self.wait_rpm(1, 0, 0, minimum_duration=1)
        # ICE provides forward thrust, which can make us think we're flying:
        self.disarm_vehicle(force=True)
        self.reboot_sitl()

        self.start_subtest("Testing throttle out in manual mode")
        self.change_mode('MANUAL')
        self.set_rc(3, 1700)
        self.wait_servo_channel_value(3, 2000)
        self.set_parameter("ICE_OPTIONS", 4)
        # remember that throttle is reversed!
        self.wait_servo_channel_value(3, 1300)
        self.change_mode('FBWA')
        self.wait_servo_channel_value(3, 2000)

        self.start_subtest("Testing automatic restart")
        # Limit start attempts to 4
        max_tries = 4
        self.set_parameter("ICE_STRT_MX_RTRY", max_tries)
        # Make the engine unable to run (by messing up the RPM sensor)
        rpm_chan = self.get_parameter("ICE_RPM_CHAN")
        self.set_parameter("ICE_RPM_CHAN", 120) # Set to a non-existent sensor
        self.set_rc(rc_engine_start_chan, 2000)
        self.wait_statustext("Uncommanded engine stop")
        self.wait_statustext("Starting engine")
        # Restore the engine
        self.set_parameter("ICE_RPM_CHAN", rpm_chan)
        # Make sure the engine continues to run for the next 30 seconds
        try:
            self.wait_statustext("Uncommanded engine stop", timeout=30)
            # The desired result is for the wait_statustext raise AutoTestTimeoutException
            raise NotAchievedException("Engine stopped unexpectedly")
        except AutoTestTimeoutException:
            pass
        self.context_stop_collecting("STATUSTEXT")

        self.start_subtest("Testing automatic starter attempt limit")
        # Try this test twice.
        # For the first run, since the engine has been running successfully in
        # the previous test for 30 seconds, the limit should reset. For the
        # second run, after commanding an engine stop, the limit should reset.
        for i in range(2):
            self.context_collect("STATUSTEXT")
            self.set_parameter("ICE_RPM_CHAN", 120) # Set to a non-existent sensor
            self.set_rc(rc_engine_start_chan, 2000)
            self.wait_statustext("Engine max crank attempts reached", check_context=True, timeout=30)
            # wait for another 30 seconds to make sure the engine doesn't restart
            self.delay_sim_time(30, reason="engine restart check")
            messages = self.context_get().collections["STATUSTEXT"]
            self.context_stop_collecting("STATUSTEXT")
            # check for the exact number of starter attempts
            attempts = 0
            for m in messages:
                if "Starting engine" == m.text:
                    attempts += 1
            if attempts != max_tries:
                raise NotAchievedException(f"Run {i+1}: Expected {max_tries} attempts, got {attempts}")
            # Command an engine stop
            self.context_collect("STATUSTEXT")
            self.set_rc(rc_engine_start_chan, 1000)
            self.wait_statustext("ignition:0", check_context=True)
            self.context_stop_collecting("STATUSTEXT")
