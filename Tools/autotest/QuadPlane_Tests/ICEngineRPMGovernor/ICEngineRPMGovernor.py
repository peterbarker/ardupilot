'''
QuadPlane autotest: ICEngineRPMGovernor

AP_FLAKE8_CLEAN
'''

from pymavlink import mavutil

from test_discovery import TestGroup
from test_discovery import testcase


class ICEngineRPMGovernor(TestGroup):
    @testcase(bucket="QuadPlaneTests1c")
    def ICEngineRPMGovernor(self):
        '''Test ICE idle and redline governor'''
        self.setup_ICEngine_vehicle()

        # allow running while disarmed
        options = int(self.get_parameter("ICE_OPTIONS"))
        options |= 1 << 2
        self.set_parameter("ICE_OPTIONS", options)

        self.start_subtest("ICEngine idle governor")
        # idle governor should work even in non-manual mode
        self.change_mode('QHOVER')
        self.run_cmd(mavutil.mavlink.MAV_CMD_DO_ENGINE_CONTROL, p1=1)

        deadband = 50
        self.set_parameter("ICE_IDLE_DB", deadband)
        # Test two RPM settings to make sure we don't pass as a fluke
        for idle_rpm in (1000, 1500):
            self.set_parameter("ICE_IDLE_RPM", idle_rpm)
            self.wait_rpm(
                1,
                idle_rpm - 1.1 * deadband,
                idle_rpm + 1.1 * deadband,
                timeout=60,
                minimum_duration=15,
            )

        self.start_subtest("ICEngine redline governor")
        self.change_mode('MANUAL')
        # The redline governor only works properly while armed
        self.wait_ready_to_arm()
        self.arm_vehicle()
        self.set_rc(3, 2000)
        for redline_rpm in (6500, 6000):
            self.set_parameter("ICE_REDLINE_RPM", redline_rpm)
            self.wait_rpm(
                1,
                redline_rpm - 2 * deadband,
                redline_rpm,
                timeout=60,
                minimum_duration=15,
            )
        self.set_rc(3, 1000)
        self.disarm_vehicle()
