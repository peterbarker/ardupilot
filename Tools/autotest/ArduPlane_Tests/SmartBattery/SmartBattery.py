'''
Plane autotest: SmartBattery

AP_FLAKE8_CLEAN
'''

from vehicle_test_suite import NotAchievedException

from test_discovery import TestGroup
from test_discovery import testcase


class SmartBattery(TestGroup):
    @testcase(bucket="PlaneTests1b")
    def SmartBattery(self):
        '''Test smart battery logging etc'''
        self.set_parameters({
            "BATT_MONITOR": 16, # Maxell battery monitor
        })

        # Must reboot sitl after setting monitor type for SMBus parameters to be set due to dynamic group
        self.reboot_sitl()
        self.set_parameters({
            "BATT_I2C_BUS": 2,      # specified in SIM_I2C.cpp
            "BATT_I2C_ADDR": 11,    # specified in SIM_I2C.cpp
        })
        self.reboot_sitl()

        self.wait_ready_to_arm()
        m = self.assert_receive_message('BATTERY_STATUS', timeout=10)
        if m.voltages_ext[0] == 65536:
            raise NotAchievedException("Flag value rather than voltage")
        if abs(m.voltages_ext[0] - 1000) > 300:
            raise NotAchievedException("Did not get good ext voltage (got=%f)" %
                                       (m.voltages_ext[0],))
        self.arm_vehicle()
        self.delay_sim_time(5, reason="battery log data to be written")
        self.disarm_vehicle()
        if not self.current_onboard_log_contains_message("BCL2"):
            raise NotAchievedException("Expected BCL2 message")
