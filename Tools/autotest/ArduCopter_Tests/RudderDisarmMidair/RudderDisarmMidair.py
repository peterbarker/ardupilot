'''
Copter autotest: RudderDisarmMidair

AP_FLAKE8_CLEAN
'''

from test_discovery import TestGroup
from test_discovery import testcase


class RudderDisarmMidair(TestGroup):
    @testcase(bucket="CopterTests2c")
    def RudderDisarmMidair(self):
        '''check disarm behaviour mid-air'''
        self.change_mode('LOITER')
        self.wait_ready_to_arm()
        self.change_mode('STABILIZE')

        # Set home current location, this gives a large home vs origin difference
        self.set_home(self.get_location())

        self.set_rc(4, 2000)
        self.wait_armed()
        self.set_rc_from_map({4: 1500, 3: 2000})
        self.wait_altitude(300, 20000, relative=True, timeout=10000)
        self.hover()
        self.set_rc(4, 1000)
        self.delay_sim_time(11, reason="confirming disarm by rudder does not trigger in flight")
        self.progress("Checking we are still armed")
        self.assert_armed()

        self.set_rc(3, 2000)
        self.wait_altitude(300, 20000, relative=True)
        self.set_rc_from_map({3: 1000, 4: 1000})
        self.progress("Checking we disarm")
        self.wait_disarmed(timeout=20)
        self.set_rc(4, 1500)
        self.arm_vehicle()
        self.set_rc(3, 1600)
        self.delay_sim_time(10, reason="vehicle to climb to RTL altitude")

        self.do_RTL(timeout=600)
