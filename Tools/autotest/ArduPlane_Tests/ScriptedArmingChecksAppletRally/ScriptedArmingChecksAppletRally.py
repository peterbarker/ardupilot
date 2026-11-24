'''
Plane autotest: ScriptedArmingChecksAppletRally

AP_FLAKE8_CLEAN
'''

from vehicle_test_suite import AltFrame

from test_discovery import TestGroup
from test_discovery import testcase


class ScriptedArmingChecksAppletRally(TestGroup):
    @testcase(bucket="PlaneTests1b")
    def ScriptedArmingChecksAppletRally(self):
        """ Applet for Arming Checks will prevent a vehicle from arming based on scripted checks
            """
        self.start_subtest("Scripted Arming Checks Applet validation")
        self.context_collect("STATUSTEXT")

        """Initialize the FC"""
        self.set_parameter("SCR_ENABLE", 1)
        self.install_applet_script_context("arming-checks.lua")
        self.reboot_sitl()
        self.wait_ekf_happy()
        self.wait_text("ArduPilot Ready", check_context=True)
        self.wait_text("Arming Checks .* loaded", timeout=30, check_context=True, regex=True)

        self.start_subsubtest("ArmCk: MAV_SYSID not set")
        self.progress("Currently SYSID is %f" % self.get_parameter('MAV_SYSID'))
        self.wait_text("ArmCk: warn: MAV_SYSID not set", timeout=30, check_context=True, regex=True)
        """ disable the SYSID check, since autotest doesn't like changing the sysid"""
        self.set_parameters({
            "ARM_SYSID": -1,
            "ARM_FOLL_SYSID": -1,
            "ARM_FOLL_SYSID_X": -1,
            "ARM_FOLL_OFS_DEF": -1,
            "ARM_P_SCALING": -1,
        })

        self.start_subsubtest("ArmCk: Rally Point must be < ARM_V_RALLY_MAX meters away")
        self.progress("Currently RALLY_LIMIT_KM is %f" % self.get_parameter('RALLY_LIMIT_KM'))
        loc = self.offset_location_ne(self.home_position_as_location(), 6500, -50)
        loc.set_alt_m(100, AltFrame.ABOVE_HOME)
        self.upload_rally_points_from_locations([loc])
        self.wait_text("warn: Rally too far", check_context=True)
        self.set_parameter("RALLY_LIMIT_KM", 7)
        self.wait_text("clear: Rally too far", check_context=True)
