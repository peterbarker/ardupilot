'''
Plane autotest: UniversalAutoLandScript

AP_FLAKE8_CLEAN
'''

from vehicle_test_suite import Location
from vehicle_test_suite import NotAchievedException

from test_discovery import TestGroup
from test_discovery import testcase


class UniversalAutoLandScript(TestGroup):
    @testcase(bucket="PlaneTests1a")
    def UniversalAutoLandScript(self):
        '''Test UniversalAutoLandScript'''
        applet_script = "UniversalAutoLand.lua"
        self.customise_SITL_commandline(["--home", "-35.362938,149.165085,585,173"])

        self.install_applet_script_context(applet_script)
        self.context_collect('STATUSTEXT')
        self.set_parameters({
            "SCR_ENABLE" : 1,
            "SCR_VM_I_COUNT" : 1000000,
            "RTL_AUTOLAND" : 2
            })
        self.reboot_sitl()
        self.wait_text("Loaded UniversalAutoLand.lua", check_context=True)
        self.set_parameters({
             "ALAND_ENABLE" : 1,
             "ALAND_WP_ALT" : 55,
             "ALAND_WP_DIST" : 400
            })
        self.wait_ready_to_arm()
        self.scripting_restart()
        self.wait_text("Scripting: restarted", check_context=True)

        self.wait_ready_to_arm()
        self.arm_vehicle()
        self.change_mode("AUTO")
        self.wait_text("Captured initial takeoff direction", check_context=True)

        self.wait_disarmed(120)
        self.progress("Check the landed heading matches takeoff")
        self.wait_heading(173, accuracy=5, timeout=1)
        loc = Location.latlon_only(-35.362938, 149.165085)
        if self.get_distance(loc, self.get_location()) > 35:
            raise NotAchievedException("Did not land close to home")
