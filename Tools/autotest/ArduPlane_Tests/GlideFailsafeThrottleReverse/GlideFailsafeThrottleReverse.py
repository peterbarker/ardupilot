'''
Plane autotest: GlideFailsafeThrottleReverse

AP_FLAKE8_CLEAN
'''

from test_discovery import TestGroup
from test_discovery import testcase


class GlideFailsafeThrottleReverse(TestGroup):
    @testcase(bucket="PlaneTests1b")
    def GlideFailsafeThrottleReverse(self):
        '''Ensure reverse thrust goes to zero, not full reverse, in the glide failsafe'''
        # the reverse thrust sim model interprets throttle as an angle
        # output: 1000 = full reverse, 1500 = zero thrust, 2000 = full forward
        self.customise_SITL_commandline([], model="plane-revthrust", wipe=True)
        self.set_parameters({
            "THR_MIN": -100,      # negative THR_MIN enables reverse thrust
            "SERVO3_MIN": 1000,
            "SERVO3_TRIM": 1500,  # zero-thrust point for a reversible ESC
            "SERVO3_MAX": 2000,
            "FS_SHORT_ACTN": 2,   # short failsafe action: glide
            "FS_LONG_ACTN": 2,    # long failsafe action: glide, so we stay in the glide
        })
        # take off in TAKEOFF mode: auto throttle avoids the reverse-thrust
        # stick mapping and never commands reverse during the climb
        self.takeoff(50, mode='TAKEOFF')
        # TAKEOFF takes no short failsafe action, CRUISE does and holds altitude on auto throttle
        self.change_mode('CRUISE')
        self.context_collect('STATUSTEXT')
        self.progress("Failing receiver to trigger glide failsafe")
        self.set_parameter("SIM_RC_FAIL", 1)  # no pulses
        self.wait_statustext("RC Short Failsafe: switched to Glide", check_context=True)
        self.wait_mode(27)  # Glide, need pymavlink support to use text name
        self.drain_mav()  # make sure we have the latest throttle output

        def within_10(got, want):
            return abs(got - want) <= 10

        # zero thrust is SERVO3_TRIM (1500); the FBWA glide drove full reverse (SERVO3_MIN, 1000)
        self.assert_servo_channel_value(3, self.get_parameter("SERVO3_TRIM"), comparator=within_10)
        self.set_parameter("SIM_RC_FAIL", 0)
        self.fly_home_land_and_disarm()
