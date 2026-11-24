'''
Copter autotest: RCOverridesClearByPilotInput

AP_FLAKE8_CLEAN
'''

from test_discovery import TestGroup
from test_discovery import testcase


class RCOverridesClearByPilotInput(TestGroup):
    @testcase(bucket="CopterTests2a")
    def RCOverridesClearByPilotInput(self):
        '''Test RC_OPTIONS bit 14 on Copter: roll/pitch/yaw/throttle each clear overrides'''
        self.set_parameters({
            "MAV_GCS_SYSID": self.mav.source_system,
            "RC12_OPTION": 46,  # RC_OVERRIDE_ENABLE aux switch
        })
        self.reboot_sitl()

        self.set_rc(12, 2000)
        self.delay_sim_time(0.2, "allow aux switch change to register")

        bit_clear_by_rc = 1 << 14

        self.start_subtest("RC_OPTIONS bit off: pilot input does not clear overrides")
        rc_options = int(self.get_parameter("RC_OPTIONS"))
        self.set_parameter("RC_OPTIONS", rc_options & ~bit_clear_by_rc)
        self._check_rc_overrides_cleared_by_pilot_input(
            trigger_ch=1, trigger_pwm=1700,
            override_ch=3, override_pwm=1300,
            expect_clear=False,
        )

        rc_options = int(self.get_parameter("RC_OPTIONS"))
        self.set_parameter("RC_OPTIONS", rc_options | bit_clear_by_rc)

        self.start_subtest("Override held while every stick stays at trim")
        self._check_rc_overrides_cleared_by_pilot_input(
            trigger_ch=None, trigger_pwm=None,
            override_ch=3, override_pwm=1300,
            expect_clear=False,
        )

        self.start_subtest("Roll input clears overrides")
        self._check_rc_overrides_cleared_by_pilot_input(
            trigger_ch=1, trigger_pwm=1700,
            override_ch=3, override_pwm=1300,
            expect_clear=True,
        )

        self.start_subtest("Pitch input clears overrides")
        self._check_rc_overrides_cleared_by_pilot_input(
            trigger_ch=2, trigger_pwm=1700,
            override_ch=1, override_pwm=1700,
            expect_clear=True,
        )

        self.start_subtest("Throttle input clears overrides")
        self._check_rc_overrides_cleared_by_pilot_input(
            trigger_ch=3, trigger_pwm=1700,
            override_ch=1, override_pwm=1700,
            expect_clear=True,
        )

        self.start_subtest("Yaw input clears overrides")
        self._check_rc_overrides_cleared_by_pilot_input(
            trigger_ch=4, trigger_pwm=1700,
            override_ch=1, override_pwm=1700,
            expect_clear=True,
        )
