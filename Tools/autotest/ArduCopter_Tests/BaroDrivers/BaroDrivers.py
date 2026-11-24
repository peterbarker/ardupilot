'''
Copter autotest: BaroDrivers

AP_FLAKE8_CLEAN
'''

from vehicle_test_suite import NotAchievedException

from test_discovery import TestGroup
from test_discovery import testcase


class BaroDrivers(TestGroup):
    @testcase(bucket="CopterTests2d")
    def BaroDrivers(self):
        '''Test Baro Drivers'''
        sensors = [
            ("MS5611", 2),
        ]
        for (name, bus) in sensors:
            self.context_push()
            if bus is not None:
                self.set_parameter("BARO_EXT_BUS", bus)
            self.set_parameter("BARO_PROBE_EXT", 1 << 2)
            self.reboot_sitl()
            self.wait_ready_to_arm()
            self.arm_vehicle()

            # insert listener to compare airspeeds:
            messages = [None, None, None]

            global count
            count = 0

            def check_pressure(mav, m):
                global count
                m_type = m.get_type()
                count += 1
                # if count > 500:
                #     if press_abs[0] is None or press_abs[1] is None:
                #         raise NotAchievedException("Not receiving messages")
                if m_type == 'SCALED_PRESSURE3':
                    off = 2
                elif m_type == 'SCALED_PRESSURE2':
                    off = 1
                elif m_type == 'SCALED_PRESSURE':
                    off = 0
                else:
                    return

                messages[off] = m

                if None in messages:
                    return
                first = messages[0]
                for msg in messages[1:]:
                    delta_press_abs = abs(first.press_abs - msg.press_abs)
                    if delta_press_abs > 0.5: # 50 Pa leeway
                        raise NotAchievedException("Press_Abs mismatch (press1=%s press2=%s)" % (first, msg))
                    delta_temperature = abs(first.temperature - msg.temperature)
                    if delta_temperature > 300:  # that's 3-degrees leeway
                        raise NotAchievedException("Temperature mismatch (t1=%s t2=%s)" % (first, msg))
            self.install_message_hook_context(check_pressure)
            self.fly_mission("copter_mission.txt", strict=False)
            if None in messages:
                raise NotAchievedException("Missing a message")

            self.context_pop()
        self.reboot_sitl()
