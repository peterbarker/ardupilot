'''
Plane autotest: AirspeedDrivers

AP_FLAKE8_CLEAN
'''

from vehicle_test_suite import NotAchievedException

from test_discovery import TestGroup
from test_discovery import testcase


class AirspeedDrivers(TestGroup):
    @testcase(bucket="PlaneTests1c")
    def AirspeedDrivers(self):
        '''Test AirSpeed drivers'''
        airspeed_sensors = [
            ("MS5525", 3, 1),
            ("DLVR", 7, 2),
            ("SITL", 100, 0),
        ]
        for (name, t, bus) in airspeed_sensors:
            self.context_push()
            if bus is not None:
                self.set_parameter("ARSPD2_BUS", bus)
            self.set_parameter("ARSPD2_TYPE", t)
            self.reboot_sitl()
            self.wait_ready_to_arm()
            self.arm_vehicle()

            # insert listener to compare airspeeds:
            airspeed = [None, None]
            # don't start testing until we've seen real speed from
            # both sensors.  This gets us out of the noise area.
            global initial_airspeed_threshold_reached
            initial_airspeed_threshold_reached = False

            def check_airspeeds(mav, m):
                global initial_airspeed_threshold_reached
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
                if airspeed[0] < 2 or airspeed[1] < 2:
                    # this mismatch can occur on takeoff, or when we
                    # smack into the ground at the end of the mission
                    return
                if not initial_airspeed_threshold_reached:
                    if not (airspeed[0] > 10 or airspeed[1] > 10):
                        return
                    initial_airspeed_threshold_reached = True
                delta = abs(airspeed[0] - airspeed[1])
                if delta > 2:
                    raise NotAchievedException("Airspeed mismatch (as1=%f as2=%f)" % (airspeed[0], airspeed[1]))
            self.install_message_hook_context(check_airspeeds)
            self.fly_mission("ap1.txt", strict=False)
            if airspeed[0] is None:
                raise NotAchievedException("Never saw an airspeed1")
            if airspeed[1] is None:
                raise NotAchievedException("Never saw an airspeed2")
            self.context_pop()
        self.reboot_sitl()
