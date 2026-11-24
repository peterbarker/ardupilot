'''
Plane autotest: TemperatureSensorRangefinder

AP_FLAKE8_CLEAN
'''

import math

from vehicle_test_suite import NotAchievedException

from test_discovery import TestGroup
from test_discovery import testcase


class TemperatureSensorRangefinder(TestGroup):
    @testcase(bucket="PlaneTests1c")
    def TemperatureSensorRangefinder(self):
        '''Test that AP_TemperatureSensor can push temperature into a rangefinder'''
        self.progress("Configure rangefinder and temperature sensor")
        self.set_parameters({
            'RNGFND1_TYPE': 100,  # SIM backend
            'TEMP1_TYPE': 8,      # SHT3X (simulated in SITL)
            'TEMP1_ADDR': 0x44,   # SHT3X I2C address
            'TEMP1_BUS': 1,       # SITL registers SHT3X on I2C bus 1
            'TEMP1_SRC': 9,       # Rangefinder
            'TEMP1_SRC_ID': 1,    # rangefinder instance 0 (source_id-1)
            'TEMP_LOG': 1,        # enable temperature sensor logging
        })
        self.reboot_sitl()

        self.context_push()
        self.set_parameter('LOG_DISARMED', 1)
        self.delay_sim_time(15, reason="waiting for temperature data to flow")
        self.context_pop()

        self.progress("Checking RFND log for valid temperature")
        dfreader = self.dfreader_for_current_onboard_log()
        found_valid_temp = False
        while True:
            m = dfreader.recv_match(type='RFND')
            if m is None:
                break
            if not math.isnan(m.Temp):
                self.progress("RFND instance %d temperature: %.1f degC" % (m.Instance, m.Temp))
                found_valid_temp = True
                break

        if not found_valid_temp:
            raise NotAchievedException("No valid temperature in RFND log messages")
