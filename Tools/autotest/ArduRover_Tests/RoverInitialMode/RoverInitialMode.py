'''
Rover autotest: RoverInitialMode

AP_FLAKE8_CLEAN
'''

from test_discovery import TestGroup
from test_discovery import testcase


class RoverInitialMode(TestGroup):
    @testcase(bucket="Rover")
    def RoverInitialMode(self):
        '''test INITIAL_MODE parameter works'''
        # from mavproxy_rc.py
        self.wait_ready_to_arm()
        mapping = [0, 1165, 1295, 1425, 1555, 1685, 1815]
        mode_ch = 8
        throttle_ch = 3
        self.set_parameter('MODE5', 3)
        self.set_rc(mode_ch, mapping[5])
        self.wait_mode('STEERING')
        self.set_rc(mode_ch, mapping[6])
        self.wait_mode('MANUAL')
        self.set_parameter("INITIAL_MODE", 1) # acro
        # stop the vehicle polling the mode switch at boot:
        self.set_parameter('FS_ACTION', 0)  # do nothing when radio fails
        self.set_rc(throttle_ch, 900)  # RC fail
        self.reboot_sitl()
        self.wait_mode("ACRO")
        self.progress("Make sure we stay in this mode")
        self.delay_sim_time(5, reason="mode persistence check")
        self.wait_mode("ACRO")
        # now change modes with a switch:
        self.set_rc(throttle_ch, 1100)
        self.delay_sim_time(3, reason="RC throttle to register")
        self.set_rc(mode_ch, mapping[5])
        self.wait_mode('STEERING')
