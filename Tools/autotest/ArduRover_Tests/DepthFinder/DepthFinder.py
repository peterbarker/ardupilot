'''
Rover autotest: DepthFinder

AP_FLAKE8_CLEAN
'''

from vehicle_test_suite import NotAchievedException

from test_discovery import TestGroup
from test_discovery import testcase


class DepthFinder(TestGroup):
    @testcase(bucket="Rover")
    def DepthFinder(self):
        '''Test multiple depthfinders for boats'''
        # Setup rangefinders
        self.customise_SITL_commandline([
            "--serial7=sim:nmea", # NMEA Rangefinder
        ])

        # RANGEFINDER_INSTANCES = [0, 2, 5]
        self.set_parameters({
            "RNGFND1_TYPE" : 17,     # NMEA must attach uart to SITL
            "RNGFND1_ORIENT" : 25,   # Set to downward facing
            "SERIAL7_PROTOCOL" : 9,  # Rangefinder on serial7
            "SERIAL7_BAUD" : 9600,   # Rangefinder specific baudrate

            "RNGFND3_TYPE" : 2,      # MaxbotixI2C
            "RNGFND3_ADDR" : 112,    # 0x70 address from SIM_I2C.cpp
            "RNGFND3_ORIENT" : 0,    # Set to forward facing, thus we should not receive DPTH messages from this one

            "RNGFND6_ADDR" : 113,    # 0x71 address from SIM_I2C.cpp
            "RNGFND6_ORIENT" : 25,   # Set to downward facing
            "RNGFND6_TYPE" : 2,      # MaxbotixI2C
        })

        self.reboot_sitl()
        self.wait_ready_to_arm()

        # should not get WATER_DEPTH messages or DPTH logs when the FRAME_CLASS is not a boat
        self.assert_not_receive_message("WATER_DEPTH", timeout=2)

        # Set FRAME_CLASS to start receiving WATER_DEPTH messages & logging DPTH
        self.set_parameters({
            "FRAME_CLASS": 2,       # Boat
        })

        # Check each rangefinder instance is in collection
        rangefinder = [None, None, None, None, None, None] # Be lazy FIXME only need [3]

        def check_rangefinder(mav, m):
            if m.get_type() != 'WATER_DEPTH':
                return

            id = m.id

            # Should not find instance 3 as it is forward facing
            if id == 2:
                raise NotAchievedException("Depthfinder Instance %i with non-downward orientation found" % (id))

            rangefinder[id] = True

            if id == 0:
                if float(m.temperature) == 0.0:
                    raise NotAchievedException("Depthfinder Instance %i NMEA with temperature not found" % (id))
            elif id == 5:
                if float(m.temperature) != 0.0:
                    raise NotAchievedException("Depthfinder Instance %i should not have temperature" % (id))

        self.wait_ready_to_arm()
        self.arm_vehicle()

        self.install_message_hook_context(check_rangefinder)
        self.drive_mission("rover1.txt", strict=False)

        if rangefinder[0] is None:
            raise NotAchievedException("Never saw Depthfinder 1")
        if rangefinder[2] is not None:
            raise NotAchievedException("Should not have found a Depthfinder 3")
        if rangefinder[5] is None:
            raise NotAchievedException("Never saw Depthfinder 6")
        if not self.current_onboard_log_contains_message("DPTH"):
            raise NotAchievedException("Expected DPTH log message")

        self.progress("Ensuring we get WATER_DEPTH at 12Hz with 2 backends")
        self.set_message_rate_hz('WATER_DEPTH', 12)
        self.assert_message_rate_hz('WATER_DEPTH', 12)
