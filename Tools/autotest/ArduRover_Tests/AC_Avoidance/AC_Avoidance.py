'''
Rover autotest: AC_Avoidance

AP_FLAKE8_CLEAN
'''

from test_discovery import TestGroup
from test_discovery import testcase


class AC_Avoidance(TestGroup):
    @testcase(bucket="Rover")
    def AC_Avoidance(self):
        '''Test AC Avoidance switch'''
        self.load_fence("rover-fence-ac-avoid.txt")
        self.set_parameters({
            "FENCE_ENABLE": 0,
            "PRX1_TYPE": 10,
            "RC10_OPTION": 40, # proximity-enable
        })
        self.reboot_sitl()
        self.wait_ready_to_arm()
        self.arm_vehicle()
        # first make sure we can breach the fence:
        self.set_rc(10, 1000)
        self.change_mode("ACRO")
        self.set_rc(3, 1550)
        self.wait_distance_to_home(25, 100000, timeout=60)
        self.change_mode("RTL")
        self.wait_statustext("Reached destination", timeout=60)
        # now enable avoidance and make sure we can't:
        self.set_rc(10, 2000)
        self.change_mode("ACRO")
        self.wait_groundspeed(0, 0.7, timeout=60)
        # watch for speed zero
        self.wait_groundspeed(0, 0.2, timeout=120)
        self.disarm_vehicle()
