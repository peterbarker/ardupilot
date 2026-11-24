'''
Copter autotest: AC_Avoidance_Beacon

AP_FLAKE8_CLEAN
'''

from vehicle_test_suite import AltFrame
from vehicle_test_suite import Location

from test_discovery import TestGroup
from test_discovery import testcase

from arducopter import SITL_START_LOCATION


class AC_Avoidance_Beacon(TestGroup):
    @testcase(bucket="CopterTests1c")
    def AC_Avoidance_Beacon(self):
        '''Test beacon avoidance slide behaviour'''
        self.context_push()
        ex = None
        try:
            self.set_parameters({
                "BCN_TYPE": 10,
                "BCN_LATITUDE": int(SITL_START_LOCATION.lat),
                "BCN_LONGITUDE": int(SITL_START_LOCATION.lng),
                "BCN_ORIENT_YAW": 45,
                "AVOID_ENABLE": 4,
            })
            self.reboot_sitl()

            self.takeoff(10, mode="LOITER")
            self.set_rc(2, 1400)
            here = self.get_location()
            here_alt_amsl = here.get_alt_m(AltFrame.ABSOLUTE)
            west_loc = Location(-35.362919, 149.165055, here_alt_amsl, AltFrame.ABSOLUTE)
            self.wait_location(west_loc, accuracy=1)
            self.reach_heading_manual(0)
            north_loc = Location(-35.362881, 149.165103, here_alt_amsl, AltFrame.ABSOLUTE)
            self.wait_location(north_loc, accuracy=1)
            self.set_rc_from_map({2: 1500, 1: 1600})
            east_loc = Location(-35.362986, 149.165227, here_alt_amsl, AltFrame.ABSOLUTE)
            self.wait_location(east_loc, accuracy=1)
            self.set_rc_from_map({1: 1500, 2: 1600})
            south_loc = Location(-35.363025, 149.165182, here_alt_amsl, AltFrame.ABSOLUTE)
            self.wait_location(south_loc, accuracy=1)
            self.set_rc(2, 1500)
            self.do_RTL()

        except Exception as e:  # noqa: BLE001
            self.print_exception_caught(e)
            ex = e
        self.context_pop()
        self.clear_fence()
        self.disarm_vehicle(force=True)
        self.reboot_sitl()
        if ex is not None:
            raise ex
