'''
Plane autotest: TakeoffTakeoff5

AP_FLAKE8_CLEAN
'''

from vehicle_test_suite import AltFrame
from vehicle_test_suite import NotAchievedException

from test_discovery import TestGroup
from test_discovery import testcase

from arduplane import SITL_START_LOCATION


class TakeoffTakeoff5(TestGroup):
    @testcase(bucket="PlaneTests1c")
    def TakeoffTakeoff5(self):
        '''Test the behaviour of a takeoff with no compass'''
        self.set_parameters({
            "COMPASS_USE": 0,
            "COMPASS_USE2": 0,
            "COMPASS_USE3": 0,
        })
        start_loc = SITL_START_LOCATION
        start_heading = 175
        self.customise_SITL_commandline(["--home=%.9f,%.9f,%.2f,%.1f" % (
            start_loc.lat,
            start_loc.lng,
            start_loc.get_alt_m(AltFrame.ABSOLUTE),
            start_heading)])
        self.reboot_sitl()
        self.change_mode("TAKEOFF")

        # waiting for the EKF to be happy won't work
        self.wait_prearm_sys_status_healthy()
        self.arm_vehicle()

        target_alt = self.get_parameter("TKOFF_ALT")
        self.wait_altitude(target_alt-5, target_alt, relative=True)

        # Wait a bit for the Takeoff altitude to settle.
        self.delay_sim_time(5, reason="takeoff altitude to settle")

        bearing_margin = 35
        loc = self.get_location()
        bearing_from_home = self.get_bearing(start_loc, loc)
        if bearing_from_home < 0:
            bearing_from_home += 360
        if abs(bearing_from_home - start_heading) > bearing_margin:
            raise NotAchievedException(f"Did not takeoff in the right direction {bearing_from_home}")

        self.fly_home_land_and_disarm()
