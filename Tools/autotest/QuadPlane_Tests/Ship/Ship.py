'''
QuadPlane autotest: Ship

AP_FLAKE8_CLEAN
'''

from test_discovery import TestGroup
from test_discovery import testcase


class Ship(TestGroup):
    @testcase(bucket="QuadPlaneTests1c")
    def Ship(self):
        '''Ensure we can take off from simulated ship'''
        self.context_push()
        self.set_parameters({
            'SIM_SHIP_ENABLE': 1,
            'SIM_SHIP_SPEED': 1,  # the default of 3 will break this test
        })
        self.change_mode('QLOITER')
        self.wait_ready_to_arm()
        self.arm_vehicle()
        self.set_rc(3, 1700)
        # self.delay_sim_time(1, "let data in log accrue")
        # self.send_debug_trap()
        # output here is a bit weird as we also receive altitude from
        # the simulated ship....
        self.wait_altitude(20, 30, relative=True)
        self.disarm_vehicle(force=True)
        self.context_pop()
        self.reboot_sitl()
