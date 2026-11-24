'''
Helicopter autotest: AirspeedDrivers

AP_FLAKE8_CLEAN
'''

from vehicle_test_suite import AltFrame
from vehicle_test_suite import Location
from vehicle_test_suite import NotAchievedException

from test_discovery import TestGroup
from test_discovery import testcase


class AirspeedDrivers(TestGroup):
    # FIXME move this & plane's version to common
    @testcase(bucket="Helicopter")
    def AirspeedDrivers(self, timeout=600):
        '''Test AirSpeed drivers'''

        # Copter's airspeed sensors are off by default
        self.set_parameters({
            "ARSPD_ENABLE": 1,
            "ARSPD_TYPE": 2,     # Analog airspeed driver
            "ARSPD_PIN": 1,      # Analog airspeed driver pin for SITL
        })
        # set the start location to CMAC to use same test script as other vehicles
        # sitl_start_location() returns this for the rest of the session,
        # so every later start_SITL() would come up at CMAC rather than
        # at the heli's own start location; put it back afterwards.
        self.context_preserve_attribute("sitl_start_loc")
        self.context_preserve_attribute("sitl_start_heading_deg")
        self.sitl_start_loc = Location(-35.362881, 149.165222, 582.000000, AltFrame.ABSOLUTE)   # CMAC
        self.sitl_start_heading_deg = 90.0
        self.customise_SITL_commandline(["--home", "%s,%s,%s,%s"
                                         % (-35.362881, 149.165222, 582.000000, 90.0)])

        # insert listener to compare airspeeds:
        airspeed = [None, None]

        def check_airspeeds(mav, m):
            m_type = m.get_type()
            if (m_type == 'NAMED_VALUE_FLOAT' and
                    m.name == 'AS2'):
                airspeed[1] = m.value
            elif m_type == 'VFR_HUD':
                airspeed[0] = m.airspeed
            else:
                return
            if airspeed[0] is None or airspeed[1] is None:
                return
            delta = abs(airspeed[0] - airspeed[1])
            if delta > 3:
                raise NotAchievedException("Airspeed mismatch (as1=%f as2=%f)" % (airspeed[0], airspeed[1]))

        airspeed_sensors = [
            ("MS5525", 3, 1),
            ("DLVR", 7, 2),
        ]
        for (name, t, bus) in airspeed_sensors:
            self.context_push()
            if bus is not None:
                self.set_parameter("ARSPD2_BUS", bus)
            self.set_parameter("ARSPD2_TYPE", t)
            self.reboot_sitl()
            self.wait_ready_to_arm()
            self.arm_vehicle()

            self.install_message_hook_context(check_airspeeds)
            self.fly_mission("ap1.txt", strict=False)

            if airspeed[0] is None:
                raise NotAchievedException("Never saw an airspeed1")
            if airspeed[1] is None:
                raise NotAchievedException("Never saw an airspeed2")
            if not self.current_onboard_log_contains_message("ARSP"):
                raise NotAchievedException("Expected ARSP log message")
            self.disarm_vehicle()
            self.context_pop()
