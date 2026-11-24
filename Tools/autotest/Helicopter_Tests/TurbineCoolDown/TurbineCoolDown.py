'''
Helicopter autotest: TurbineCoolDown

AP_FLAKE8_CLEAN
'''

import operator

from vehicle_test_suite import NotAchievedException

from test_discovery import TestGroup
from test_discovery import testcase


class TurbineCoolDown(TestGroup):
    @testcase(bucket="Helicopter")
    def TurbineCoolDown(self, timeout=200):
        """Check Turbine Cool Down Feature"""
        self.context_push()
        # set option for Turbine
        RAMP_TIME = 4
        SETPOINT = 66
        IDLE = 15
        COOLDOWN_TIME = 5
        self.set_parameters({"RC6_OPTION": 161,
                             "H_RSC_RAMP_TIME": RAMP_TIME,
                             "H_RSC_SETPOINT": SETPOINT,
                             "H_RSC_IDLE": IDLE,
                             "H_RSC_CLDWN_TIME": COOLDOWN_TIME})
        self.set_rc_from_map({3: 1000, 8: 1000})

        self.progress("Starting turbine")
        self.wait_ready_to_arm()
        self.context_collect("STATUSTEXT")
        self.arm_vehicle()

        self.set_rc(6, 2000)
        self.wait_statustext('Turbine startup', check_context=True)

        # Engage interlock to run up to head speed
        self.set_rc(8, 2000)

        # Check throttle gets to setpoint
        expected_thr = SETPOINT * 0.01 * 1000 + 1000 - 1 # servo end points are 1000 to 2000
        self.wait_servo_channel_value(8, expected_thr, timeout=RAMP_TIME+1, comparator=operator.ge)

        self.progress("Checking cool down behaviour, idle x 1.5")
        self.set_rc(8, 1000)
        tstart = self.get_sim_time()
        expected_thr = IDLE * 1.5 * 0.01 * 1000 + 1000 + 1
        self.wait_servo_channel_value(8, expected_thr, timeout=2, comparator=operator.le)

        # Check that the throttle drops to idle after cool down time
        expected_thr = IDLE * 0.01 * 1000 + 1000 + 1
        self.wait_servo_channel_value(8, expected_thr, timeout=COOLDOWN_TIME+1, comparator=operator.le)

        measured_time = self.get_sim_time() - tstart
        if (abs(measured_time - COOLDOWN_TIME) > 1.0):
            raise NotAchievedException('Throttle did not reduce to idle within H_RSC_CLDWN_TIME')

        self.set_rc(6, 1000)
        self.wait_disarmed(timeout=20)
        self.context_pop()
