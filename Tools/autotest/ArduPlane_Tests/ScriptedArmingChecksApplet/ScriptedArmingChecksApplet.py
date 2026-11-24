'''
Plane autotest: ScriptedArmingChecksApplet

AP_FLAKE8_CLEAN
'''

from test_discovery import TestGroup
from test_discovery import testcase


class ScriptedArmingChecksApplet(TestGroup):
    @testcase(bucket="PlaneTests1b")
    def ScriptedArmingChecksApplet(self):
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
        self.set_parameter("ARM_SYSID", -1)

        ''' This check comes first since its part of the standard parameters - an invalid scaling speed'''
        self.start_subsubtest("ArmCk: SCALING_SPEED close to AIRSPEED_CRUISE")
        self.assert_prearm_failure("fail: Scaling spd", other_prearm_failures_fatal=False)
        self.set_parameter("SCALING_SPEED", 22)
        self.wait_text("clear: Scaling spd", check_context=True)

        self.start_subsubtest("ArmCk: FOLL_SYSID must be set if FOLL_ENABLE = 1")
        self.set_parameter("FOLL_ENABLE", 1)
        self.set_parameter("FOLL_OFS_X", 10)
        self.assert_prearm_failure("FOLL_SYSID not set", other_prearm_failures_fatal=False)
        self.set_parameter("FOLL_SYSID", 3)
        self.wait_text("clear: FOLL_SYSID not set", check_context=True)
        self.set_parameter("FOLL_SYSID", -1)

        self.start_subsubtest("ArmCk: FOLL_OFS_[XYZ] must be set if FOLL_ENABLE = 1")
        self.set_parameter("FOLL_OFS_X", 0)
        self.assert_prearm_failure("FOLL_OFS_[XYZ] = 0", other_prearm_failures_fatal=False)
        self.set_parameter("FOLL_OFS_X", 10)
        self.wait_text("clear: FOLL_OFS_[XYZ] = 0", check_context=True)
        self.set_parameter("FOLL_OFS_X", 0)
        self.assert_prearm_failure("FOLL_OFS_[XYZ] = 0", other_prearm_failures_fatal=False)
        self.set_parameter("FOLL_OFS_Y", 10)
        self.wait_text("clear: FOLL_OFS_[XYZ] = 0", check_context=True)
        self.set_parameter("FOLL_OFS_Y", 0)
        self.assert_prearm_failure("FOLL_OFS_[XYZ] = 0", other_prearm_failures_fatal=False)
        self.set_parameter("FOLL_OFS_Z", 10)
        self.wait_text("clear: FOLL_OFS_[XYZ] = 0", check_context=True)
        """ clear these checks to make the context cleaner for subsequent tests """
        self.set_parameters({
            "ARM_SYSID": -1,
            "ARM_FOLL_SYSID": -1,
            "ARM_FOLL_SYSID_X": -1,
            "ARM_FOLL_OFS_DEF": -1,
        })

        self.start_subsubtest("ArmCk: RTL_ALTITUDE must be legal")
        self.progress("Currently ARM_P_RTL_ALT is %f" % self.get_parameter('ARM_P_RTL_ALT'))
        self.set_parameter("RTL_ALTITUDE", 150)
        self.assert_prearm_failure("ArmCk: fail: RTL_ALTITUDE too high", other_prearm_failures_fatal=False)
        self.set_parameter("RTL_ALTITUDE", 120)
        self.wait_text("clear: RTL_ALT", check_context=True)

        self.start_subsubtest("ArmCk: RTL_CLIMB_MIN must be legal")
        self.set_parameter("RTL_CLIMB_MIN", 150)
        self.wait_text("ArmCk: warn: RTL_CLIMB_MIN too high", check_context=True)
        self.set_parameter("RTL_CLIMB_MIN", 120)
        self.wait_text("clear: RTL_CLIMB_MIN too high", check_context=True)

        self.start_subsubtest("ArmCk: AIRSPEED stall < min < cruise < max")
        ''' Airspeed parameter start out as
        AIRSPEED_STALL = 0
        AIRSPEED_MIN = 10
        AIRSPEED_CRUISE = 22
        AIRSPEED_MAX = 30
        '''
        self.start_subsubtest("ArmCk: AIRSPEED max < others")
        self.set_parameter("AIRSPEED_MAX", 5)
        self.assert_prearm_failure("ArmCk: fail: stall < min", other_prearm_failures_fatal=False)
        self.set_parameter("AIRSPEED_MAX", 30)
        self.wait_text("clear: stall < min", check_context=True)
        self.start_subsubtest("ArmCk: AIRSPEED cruise < min")
        self.set_parameter("AIRSPEED_MIN", 25)
        self.assert_prearm_failure("ArmCk: fail: stall < min", other_prearm_failures_fatal=False)
        self.set_parameter("AIRSPEED_MIN", 10)
        self.wait_text("clear: stall < min", check_context=True)
        self.start_subsubtest("ArmCk: AIRSPEED cruise > max")
        self.set_parameter("AIRSPEED_CRUISE", 40)
        self.set_parameter("SCALING_SPEED", 40)
        self.assert_prearm_failure("ArmCk: fail: stall < min", other_prearm_failures_fatal=False)
        self.set_parameter("AIRSPEED_CRUISE", 22)
        self.set_parameter("SCALING_SPEED", 22)
        self.wait_text("clear: stall < min", check_context=True)

        self.start_subsubtest("ArmCk: AIRSPEED_MIN must be > AIRSPEED_STALL")
        ''' only applies if AIRSPEED_STALL is non zero'''
        self.set_parameter("AIRSPEED_STALL", 2)
        self.wait_text("ArmCk: info: Min Speed not", check_context=True)
        self.set_parameter("AIRSPEED_STALL", 8)

        self.start_subsubtest("ArmCk: Mount SYSID must not match FOLL_SYSID")
        self.set_parameter("ARM_SYSID", -1)
        ''' to fail the check MNTx_SYSID_DFLT must be non zero but not= FOLL_SYSID'''
        self.set_parameter("MNT1_SYSID_DFLT", 1)
        ''' Need a healthy camera mount defined for this to work '''
        self.setup_servo_mount()
        self.progress("rebooting to enable MNT1")
        self.reboot_sitl() # to handle MNT_TYPE changing
        self.wait_ekf_happy()
        self.wait_text("ArduPilot Ready", check_context=True)
        self.wait_text("Arming Checks .* loaded", timeout=30, check_context=True, regex=True)
        self.progress("Currently FOLL_SYSID is %f" % self.get_parameter('FOLL_SYSID'))
        self.progress("Currently MNT1_SYSID_DFLT is %f" % self.get_parameter('MNT1_SYSID_DFLT'))
        self.set_parameter("ARM_SYSID", -1)
        self.progress("Currently FOLL_SYSID is %f" % self.get_parameter('FOLL_SYSID'))
        self.progress("Currently MNT1_SYSID_DFLT is %f" % self.get_parameter('MNT1_SYSID_DFLT'))
        self.wait_text("warn: MNTx_SYSID != FOLL", check_context=True)

        self.start_subsubtest("ArmCk: Fence must be enabled or autoenabled (warning)")
        self.reboot_sitl()
        self.wait_ekf_happy()
        self.wait_text("ArduPilot Ready", check_context=True)
        self.wait_text("Arming Checks .* loaded", timeout=30, check_context=True, regex=True)
        self.load_fence("CMAC-fence.txt")
        self.wait_statustext("warn: Fence not enabled", check_context=True)
        self.set_parameter("FENCE_ENABLE", 1)
        self.wait_statustext("clear: Fence not enabled", check_context=True)
        self.set_parameter("FENCE_ENABLE", 0)
        self.wait_statustext("warn: Fence not enabled", check_context=True)
        self.set_parameter("FENCE_AUTOENABLE", 1)
        self.wait_text("clear: Fence not enabled", check_context=True)
        self.set_parameter("FENCE_AUTOENABLE", 0)
        self.wait_text("warn: Fence not enabled", check_context=True)
