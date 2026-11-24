'''
Sub autotest: INA3221

AP_FLAKE8_CLEAN
'''

from vehicle_test_suite import NotAchievedException

from test_discovery import TestGroup
from test_discovery import testcase


class INA3221(TestGroup):
    @testcase(bucket="Sub")
    def INA3221(self):
        '''test INA3221 driver'''
        self.set_parameters({
            "BATT2_MONITOR": 30,
            "BATT3_MONITOR": 30,
            "BATT4_MONITOR": 30,
        })
        self.reboot_sitl()
        self.set_parameters({
            "BATT2_I2C_ADDR": 0x42, # address defined in libraries/SITL/SIM_I2C.cpp
            "BATT2_I2C_BUS": 1,
            "BATT2_CHANNEL": 1,

            "BATT3_I2C_ADDR": 0x42,
            "BATT3_I2C_BUS": 1,
            "BATT3_CHANNEL": 2,

            "BATT4_I2C_ADDR": 0x42,
            "BATT4_I2C_BUS": 1,
            "BATT4_CHANNEL": 3,
        })
        self.reboot_sitl()

        seen_1 = False
        seen_3 = False
        tstart = self.get_sim_time()
        while not (seen_1 and seen_3):
            m = self.assert_receive_message('BATTERY_STATUS')
            if self.get_sim_time() - tstart > 10:
                # expected to take under 1 simulated second, but don't hang if
                # e.g. the driver gets stuck
                raise NotAchievedException("INA3221 status timeout")
            if m.id == 1:
                self.assert_message_field_values(m, {
                    # values close to chip limits
                    "voltages[0]": int(25 * 1000), # millivolts
                    "current_battery": int(160 * 100), # centi-amps
                }, epsilon=10) # allow rounding
                seen_1 = True
            # id 2 is the first simulated battery current/voltage
            if m.id == 3:
                self.assert_message_field_values(m, {
                    # values different from above to test channel switching
                    "voltages[0]": int(3.14159 * 1000), # millivolts
                    "current_battery": int(2.71828 * 100), # centi-amps
                }, epsilon=10) # allow rounding
                seen_3 = True
