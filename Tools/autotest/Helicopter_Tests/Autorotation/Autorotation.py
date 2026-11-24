'''
Helicopter autotest: Autorotation

AP_FLAKE8_CLEAN
'''

import operator

from vehicle_test_suite import NotAchievedException

from test_discovery import TestGroup
from test_discovery import testcase


class Autorotation(TestGroup):
    @testcase(bucket="Helicopter")
    def Autorotation(self, timeout=600):
        """Check engine-out behaviour"""
        self.context_push()
        start_alt = 100 # metres
        self.set_parameters({
            "AROT_ENABLE": 1,
            "H_RSC_AROT_ENBL": 1,
            "H_COL_LAND_MIN" : -2.0
        })
        bail_out_time = self.get_parameter('H_RSC_AROT_RUNUP')
        self.change_mode('POSHOLD')
        self.set_rc_from_map({3: 1000, 8: 1000})
        self.wait_ready_to_arm()
        self.arm_vehicle()
        self.set_rc(8, 2000)
        self.progress("wait for rotor runup to complete")
        self.wait_servo_channel_value(8, 1659, timeout=10)
        self.delay_sim_time(20, reason="rotor runup to complete")
        self.set_rc(3, 2000)
        self.wait_altitude(start_alt - 1,
                           (start_alt + 5),
                           relative=True,
                           timeout=timeout)
        self.context_collect('STATUSTEXT')

        # Reset collective to enter hover
        self.set_rc(3, 1500)

        # Change to the autorotation flight mode
        self.progress("Triggering autorotate mode")
        self.change_mode('AUTOROTATE')

        # Disengage the interlock to remove power
        self.set_rc(8, 1000)

        # Ensure we have progressed through the mode's state machine
        self.wait_statustext("Glide Phase", check_context=True)

        self.progress("Testing bailout from autorotation")
        self.set_rc(8, 2000)
        # See if the output ramps to a value close to expected with the prescribed time
        self.wait_servo_channel_value(8, 1659, timeout=bail_out_time+1, comparator=operator.ge)

        # Successfully bailed out, disengage the interlock and allow autorotation to progress
        self.set_rc(8, 1000)
        self.wait_statustext(r"SIM Hit ground at ([0-9.]+) m/s",
                             check_context=True,
                             regex=True)
        speed = float(self.re_match.group(1))
        if speed > 30:
            raise NotAchievedException("Hit too hard")

        # Set throttle low to trip auto disarm
        self.set_rc(3, 1000)

        self.wait_disarmed()
        self.context_pop()
