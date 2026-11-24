'''
Plane autotest: GlideFailsafeThrottle

AP_FLAKE8_CLEAN
'''

import operator

from test_discovery import TestGroup
from test_discovery import testcase


class GlideFailsafeThrottle(TestGroup):
    @testcase(bucket="PlaneTests1b")
    def GlideFailsafeThrottle(self):
        '''Ensure throttle is forced to zero in the glide failsafe and pilot authority returns on recovery'''
        self.set_parameters({
            "FS_SHORT_ACTN": 2,  # short failsafe action: glide
            "FS_LONG_ACTN": 2,   # long failsafe action: glide, so we stay in the glide
            "THR_MIN": 20,       # non-zero so a zeroed throttle is distinct from throttle min
        })
        self.takeoff(50, mode='TAKEOFF')
        # TAKEOFF takes no short failsafe action, CRUISE does and holds altitude on auto throttle
        self.change_mode('CRUISE')
        self.context_collect('STATUSTEXT')
        self.progress("Failing receiver to trigger glide failsafe")
        self.set_parameter("SIM_RC_FAIL", 1)  # no pulses
        self.wait_statustext("RC Short Failsafe: switched to Glide", check_context=True)
        self.wait_mode(27)  # Glide, need pymavlink support to use text name
        self.drain_mav()  # make sure we have the latest throttle output
        # throttle must be forced to zero (SERVO3_MIN), not held at THR_MIN
        self.assert_servo_channel_value(3, self.get_parameter("SERVO3_MIN"))

        self.progress("Waiting for long failsafe, we should stay in the glide")
        self.wait_statustext("RC Long Failsafe On", check_context=True)
        self.assert_mode(27)

        self.progress("Restoring receiver, long failsafe does not restore the mode")
        self.set_parameter("SIM_RC_FAIL", 0)
        self.wait_statustext("RC Long Failsafe Cleared", check_context=True)
        self.assert_mode(27)

        self.progress("Pilot must be able to leave the glide and regain throttle")
        self.change_mode("FBWA")
        self.set_rc(3, 1800)
        self.wait_servo_channel_value(3, 1500, comparator=operator.ge, timeout=5)
        self.set_rc(3, 1500)
        self.fly_home_land_and_disarm()
