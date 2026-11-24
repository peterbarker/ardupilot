'''
Copter autotest: test_EKF3_option_disable_lane_switch

AP_FLAKE8_CLEAN
'''

from vehicle_test_suite import NotAchievedException

from test_discovery import TestGroup
from test_discovery import testcase


class test_EKF3_option_disable_lane_switch(TestGroup):
    @testcase(bucket="CopterTests1a")
    def test_EKF3_option_disable_lane_switch(self):
        '''Test that EK3_OPTION disables lane switching, and EK3_PRIMARY forces switch when re-enabled'''

        self.set_parameters({
            "EK3_ENABLE": 1,
            "EK2_ENABLE": 0,
            "AHRS_EKF_TYPE": 3,
            "EK3_IMU_MASK": 3,       # Use IMU0 and IMU1
            "EK3_OPTIONS": 2,         # Disable lane switching
            "EK3_PRIMARY": 0,        # Start with lane 0
        })

        self.reboot_sitl()

        self.lane_switches = []

        # Hook to track STATUSTEXT messages for EKF lane switches
        def statustext_hook(mav, message):
            if message.get_type() != 'STATUSTEXT':
                return
            if message.text.startswith("EKF3 primary changed:"):
                try:
                    lane = int(message.text.strip().split(":")[-1])
                    self.lane_switches.append(lane)
                except ValueError:
                    pass  # ignore malformed messages

        self.install_message_hook_context(statustext_hook)

        self.takeoff(50, mode='ALT_HOLD')

        self.delay_sim_time(5, reason="EKF to stabilise after takeoff")

        ####################################################################################
        self.start_subtest("Ensure no lane switch occurs with EK3_OPTIONS = 2")
        self.context_collect("STATUSTEXT")
        self.set_parameters({
            "INS_ACCOFFS_X" : 5
        })

        self.delay_sim_time(10, reason="confirming no lane switch occurs")  # Wait to confirm no switch
        if self.lane_switches:
            raise NotAchievedException(f"Unexpected lane switch occurred: {self.lane_switches}")
        self.progress("Success: No lane switch occurred with EK3_OPTIONS = 2")
        self.context_clear_collection("STATUSTEXT")
        self.set_parameters({
            "INS_ACCOFFS_X" : 0.01,
        })

        ####################################################################################
        self.start_subtest("EK3_PRIMARY = 1 (expect switch)")

        self.context_collect("STATUSTEXT")
        self.set_parameters({
            "EK3_PRIMARY": 1,
        })

        # Wait for automatic lane switch to occur
        self.wait_statustext(
            text="EKF3 primary changed:1",
            timeout=30,
            check_context=True
        )

        self.context_clear_collection("STATUSTEXT")
        self.disarm_vehicle(force=True)
