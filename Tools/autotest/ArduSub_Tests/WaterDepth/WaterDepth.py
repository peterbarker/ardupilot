'''
Sub autotest: WaterDepth

AP_FLAKE8_CLEAN
'''

from test_discovery import TestGroup
from test_discovery import testcase


class WaterDepth(TestGroup):
    @testcase(bucket="Sub")
    def WaterDepth(self):
        """Check WATER_DEPTH MAVLink message support for ArduSub"""

        # Setup rangefinders
        self.customise_SITL_commandline([
            "--serial7=sim:nmea", # NMEA Rangefinder
        ])

        self.set_parameters({
            "RNGFND1_TYPE" : 17,     # NMEA must attach uart to SITL
            "RNGFND1_ORIENT" : 25,   # Set to downward facing
            "RNGFND1_MIN": 0.10,
            "RNGFND1_MAX": 30.00,
            "SERIAL7_PROTOCOL" : 9,  # Rangefinder on serial7
            "SERIAL7_BAUD" : 9600,   # Rangefinder specific baudrate
        })

        self.reboot_sitl()
        self.set_rc_default()
        self.wait_ready_to_arm()

        self.set_message_rate_hz('WATER_DEPTH', 2)

        self.progress("Arming vehicle for WATER_DEPTH test")
        self.arm_vehicle()

        # wait for at least one WATER_DEPTH message
        self.progress("Waiting for WATER_DEPTH message")
        self.assert_receive_message('WATER_DEPTH', timeout=20)

        # assert the message rate is correct
        self.progress("Checking WATER_DEPTH message rate")
        self.assert_message_rate_hz('WATER_DEPTH', 2)

        self.disarm_vehicle()
