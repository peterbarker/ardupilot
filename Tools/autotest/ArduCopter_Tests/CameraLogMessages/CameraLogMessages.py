'''
Copter autotest: CameraLogMessages

AP_FLAKE8_CLEAN
'''

import math

from pymavlink import mavextra

from vehicle_test_suite import NotAchievedException

from test_discovery import TestGroup
from test_discovery import testcase


class CameraLogMessages(TestGroup):
    @testcase(bucket="CopterTests2c")
    def CameraLogMessages(self):
        '''ensure Camera log messages are good'''
        self.set_parameter("RC12_OPTION", 9) # CameraTrigger
        self.set_parameter("CAM1_TYPE", 1)   # Camera with servo trigger
        self.reboot_sitl() # needed for RC12_OPTION to take effect

        gpis = []
        gps_raws = []
        attitudes = []

        self.takeoff(10, mode='GUIDED')
        self.set_rc(12, 2000)
        gpis.append(self.assert_receive_message('GLOBAL_POSITION_INT'))
        gps_raws.append(self.assert_receive_message('GPS_RAW_INT'))
        attitudes.append(self.assert_receive_message('ATTITUDE'))
        self.set_rc(12, 1000)

        self.fly_guided_move_local(0, 0, 20)

        self.set_rc(12, 2000)
        gpis.append(self.assert_receive_message('GLOBAL_POSITION_INT'))
        gps_raws.append(self.assert_receive_message('GPS_RAW_INT'))
        attitudes.append(self.assert_receive_message('ATTITUDE'))
        self.set_rc(12, 1000)

        self.hover()
        self.change_mode('LOITER')
        self.set_rc_from_map({1: 1000, 2: 1000})
        self.delay_sim_time(5, reason="vehicle to slow down in loiter")
        self.set_rc(12, 2000)
        gpis.append(self.assert_receive_message('GLOBAL_POSITION_INT'))
        gps_raws.append(self.assert_receive_message('GPS_RAW_INT'))
        attitudes.append(self.assert_receive_message('ATTITUDE'))
        self.set_rc_from_map({12: 1000, 1: 1500, 2: 1500})

        self.do_RTL()

        dfreader = self.dfreader_for_current_onboard_log()

        for i in range(len(gpis)):
            gpi = gpis[i]
            gps_raw = gps_raws[i]
            attitude = attitudes[i]
            m = dfreader.recv_match(type="CAM")

            things = [
                ["absalt", gpi.alt*0.001, m.Alt],
                ["relalt", gpi.relative_alt*0.001, m.RelAlt],
                ["gpsalt", gps_raw.alt*0.001, m.GPSAlt],  # use GPS_RAW here?
                ["R", math.degrees(attitude.roll), m.R],
                ["P", math.degrees(attitude.pitch), m.P],
                ["Y", mavextra.wrap_360(math.degrees(attitude.yaw)), m.Y],
            ]
            for (name, want, got) in things:
                if abs(got - want) > 1:
                    raise NotAchievedException(f"Incorrect {name} {want=} {got=}")
                self.progress(f"{name} {want=} {got=}")

            want = gpi.relative_alt*0.001
            got = m.RelAlt
            if abs(got - want) > 1:
                raise NotAchievedException(f"Incorrect relalt {want=} {got=}")
