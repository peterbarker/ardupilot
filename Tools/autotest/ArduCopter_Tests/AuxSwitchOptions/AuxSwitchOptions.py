'''
Copter autotest: AuxSwitchOptions

AP_FLAKE8_CLEAN
'''

from test_discovery import TestGroup
from test_discovery import testcase


class AuxSwitchOptions(TestGroup):
    @testcase(bucket="CopterTests1c")
    def AuxSwitchOptions(self):
        '''Test random aux mode options'''
        # establish home before exercising the clear-waypoints switch:
        # when home is set (EKF origin arriving on a loaded machine
        # well into the test) the firmware writes home back into the
        # mission as item 0, and a clear racing that ends with
        # MIS_TOTAL=1:
        #     Unexpected count got=1 want=0
        self.wait_ready_to_arm()
        self.set_parameter("RC7_OPTION", 58) # clear waypoints
        self.load_mission("copter_loiter_to_alt.txt")
        self.set_rc(7, 1000)
        self.assert_mission_count(5)
        self.progress("Clear mission")
        self.set_rc(7, 2000)
        self.delay_sim_time(1, reason="switch to debounce")  # allow switch to debounce
        self.assert_mission_count(0)
        self.set_rc(7, 1000)
        self.set_parameter("RC7_OPTION", 24) # reset mission
        self.delay_sim_time(2, reason="parameter change to take effect")
        self.load_mission("copter_loiter_to_alt.txt")
        set_wp = 4
        self.set_current_waypoint(set_wp)
        self.wait_current_waypoint(set_wp, timeout=10)
        self.progress("Reset mission")
        self.set_rc(7, 2000)
        self.wait_current_waypoint(0, timeout=10)
        self.set_rc(7, 1000)
