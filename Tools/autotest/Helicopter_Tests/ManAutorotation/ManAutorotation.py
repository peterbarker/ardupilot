'''
Helicopter autotest: ManAutorotation

AP_FLAKE8_CLEAN
'''

from vehicle_test_suite import NotAchievedException

from test_discovery import TestGroup
from test_discovery import testcase


class ManAutorotation(TestGroup):
    @testcase(bucket="Helicopter")
    def ManAutorotation(self, timeout=600):
        """Check autorotation power recovery behaviour"""
        RSC_CHAN = 8

        def check_rsc_output(self, throttle, timeout):
            # Check we get a sensible throttle output
            expected_pwm = int(throttle * 0.01 * 1000 + 1000)

            # Help out the detection by accepting some margin
            margin = 2

            # See if the output ramps to a value close to expected with the prescribed time
            self.wait_servo_channel_in_range(RSC_CHAN, expected_pwm-margin, expected_pwm+margin, timeout=timeout)

        def TestAutorotationConfig(self, rsc_idle, arot_ramp_time, arot_idle, cool_down):
            RAMP_TIME = 10
            RUNUP_TIME = 15
            AROT_RUNUP_TIME = arot_ramp_time + 4
            RSC_SETPOINT = 66
            self.set_parameters({
                "H_RSC_AROT_ENBL": 1,
                "H_RSC_AROT_RAMP": arot_ramp_time,
                "H_RSC_AROT_RUNUP": AROT_RUNUP_TIME,
                "H_RSC_AROT_IDLE": arot_idle,
                "H_RSC_RAMP_TIME": RAMP_TIME,
                "H_RSC_RUNUP_TIME": RUNUP_TIME,
                "H_RSC_IDLE": rsc_idle,
                "H_RSC_SETPOINT": RSC_SETPOINT,
                "H_RSC_CLDWN_TIME": cool_down
            })

            # Check the RSC config so we know what to expect on the throttle output
            if self.get_parameter("H_RSC_MODE") != 2:
                self.set_parameter("H_RSC_MODE", 2)
                self.reboot_sitl()

            self.change_mode('POSHOLD')
            self.set_rc_from_map({3: 1000, 8: 1000})
            self.wait_ready_to_arm()
            self.arm_vehicle()
            self.set_rc(8, 2000)
            self.progress("wait for rotor runup to complete")
            check_rsc_output(self, RSC_SETPOINT, RUNUP_TIME+1)

            self.delay_sim_time(20, reason="rotor runup to complete")
            self.set_rc(3, 2000)
            self.wait_altitude(100,
                               105,
                               relative=True,
                               timeout=timeout)
            self.context_collect('STATUSTEXT')
            self.change_mode('STABILIZE')

            self.progress("Triggering manual autorotation by disabling interlock")
            self.set_rc_from_map({3: 1000, 8: 1000})

            self.wait_statustext(r"RSC: In Autorotation", check_context=True)

            # Check we are using the correct throttle output. This should happen instantly on ramp down.
            idle_thr = rsc_idle
            if (arot_idle > 0):
                idle_thr = arot_idle

            check_rsc_output(self, idle_thr, 1)

            self.progress("RSC is outputting correct idle throttle")

            # Wait to establish autorotation.
            self.delay_sim_time(2, reason="autorotation to establish")

            # Re-engage interlock to start bailout sequence
            self.set_rc(8, 2000)

            # Ensure we see the bailout state
            self.wait_statustext("RSC: Bailing Out", check_context=True)

            # Check we are back up to flight throttle. Autorotation ramp up time should be used
            check_rsc_output(self, RSC_SETPOINT, arot_ramp_time+1)

            # Give time for engine to power up
            self.set_rc(3, 1400)
            self.delay_sim_time(2, reason="engine to power up")

            self.progress("in-flight power recovery")
            self.set_rc(3, 1500)
            self.delay_sim_time(5, reason="power recovery to stabilise")

            # Initiate autorotation again
            self.set_rc_from_map({3: 1000, 8: 1000})

            self.wait_statustext(r"SIM Hit ground at ([0-9.]+) m/s",
                                 check_context=True,
                                 regex=True)
            speed = float(self.re_match.group(1))
            if speed > 30:
                raise NotAchievedException("Hit too hard")

            # Check that cool down is still used correctly if set
            # First wait until we are out of the autorotation state
            self.wait_statustext("RSC: Autorotation Stopped")
            if (cool_down > 0):
                check_rsc_output(self, rsc_idle*1.5, cool_down)

            # Verify RSC output resets to RSC_IDLE after land complete
            check_rsc_output(self, rsc_idle, 20)
            self.wait_disarmed()

        # We test the bailout behavior of two different configs
        # First we test config with a regular throttle curve
        self.start_subtest("testing autorotation with throttle curve config")
        self.context_push()
        TestAutorotationConfig(self, rsc_idle=5.0, arot_ramp_time=2.0, arot_idle=0, cool_down=0)

        # Now we test a config that would be used with an ESC with internal governor and an autorotation window
        self.start_subtest("testing autorotation with ESC autorotation window config")
        TestAutorotationConfig(self, rsc_idle=0.0, arot_ramp_time=0.0, arot_idle=20.0, cool_down=0)

        # Check rsc output behavior when using the cool down feature
        self.start_subtest("testing autorotation with cool down enabled and zero autorotation idle")
        TestAutorotationConfig(self, rsc_idle=5.0, arot_ramp_time=2.0, arot_idle=0, cool_down=5.0)

        self.start_subtest("testing that H_RSC_AROT_IDLE is used over RSC_IDLE when cool down is enabled")
        TestAutorotationConfig(self, rsc_idle=5.0, arot_ramp_time=2.0, arot_idle=10, cool_down=5.0)

        self.context_pop()
