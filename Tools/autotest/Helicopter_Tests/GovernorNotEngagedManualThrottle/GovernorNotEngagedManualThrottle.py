'''
Helicopter autotest: GovernorNotEngagedManualThrottle

AP_FLAKE8_CLEAN
'''

from pymavlink import mavutil

from vehicle_test_suite import NotAchievedException

from test_discovery import TestGroup
from test_discovery import testcase


class GovernorNotEngagedManualThrottle(TestGroup):
    @testcase(bucket="Helicopter")
    def GovernorNotEngagedManualThrottle(self):
        '''check runup complete and land-complete clear in manual throttle modes when governor never engages'''
        self.customise_SITL_commandline(
            [],
            defaults_filepath=self.model_defaults_filepath('heli-gas'),
            model="heli-gas",
            wipe=True,
        )
        # AutoThrottle RSC mode with the rotor speed sensor removed;
        # without RPM feedback the governor can never engage:
        self.set_parameters({
            "H_RSC_MODE": 4,
            "RPM1_TYPE": 0,
        })
        self.reboot_sitl()

        self.context_collect('STATUSTEXT')
        self.context_set_message_rate_hz(id=mavutil.mavlink.MAVLINK_MSG_ID_EXTENDED_SYS_STATE, rate_hz=1)

        self.change_mode('ALT_HOLD')
        self.wait_ready_to_arm()
        self.arm_vehicle()
        self.progress("Raising rotor speed")
        self.set_rc(8, 2000)

        # wait beyond the rotor ramp and runup timers:
        runup_time = (self.get_parameter("H_RSC_RAMP_TIME") +
                      self.get_parameter("H_RSC_RUNUP_TIME"))
        self.delay_sim_time(runup_time + 10, reason="rotor ramp and runup timers to expire")

        # in a non-manual-throttle mode runup must not be declared
        # complete until the governor engages:
        if self.statustext_in_collections("Runup Complete") is not None:
            raise NotAchievedException(
                "Runup completed without governor engaged in non-manual throttle mode")

        self.progress("Switching to a manual throttle mode")
        self.change_mode('STABILIZE')
        self.wait_statustext("Governor Failed to Engage when Runup Completed", check_context=True, timeout=30)

        self.progress("Take off and check land-complete is cleared")
        self.assert_extended_sys_state(
            vtol_state=mavutil.mavlink.MAV_VTOL_STATE_MC,
            landed_state=mavutil.mavlink.MAV_LANDED_STATE_ON_GROUND,
        )
        self.set_rc(3, 1700)
        self.wait_altitude(5, 30, relative=True, timeout=60)
        self.hover()
        self.wait_extended_sys_state(
            vtol_state=mavutil.mavlink.MAV_VTOL_STATE_MC,
            landed_state=mavutil.mavlink.MAV_LANDED_STATE_IN_AIR,
            timeout=10,
        )

        self.do_RTL()
