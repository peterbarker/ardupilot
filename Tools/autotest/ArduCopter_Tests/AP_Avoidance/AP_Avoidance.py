'''
Copter autotest: AP_Avoidance

AP_FLAKE8_CLEAN
'''

from vehicle_test_suite import AltFrame

from test_discovery import TestGroup
from test_discovery import testcase


class AP_Avoidance(TestGroup):
    @testcase(bucket="CopterTests1e")
    def AP_Avoidance(self):
        '''ADSB-based avoidance'''
        self.set_parameters({
            "AVD_ENABLE": 1,
            "ADSB_TYPE": 1,  # mavlink
            "AVD_F_ACTION": 2,  # climb or descend
        })
        self.reboot_sitl()

        self.wait_ready_to_arm()

        here = self.get_location()

        self.context_push()

        self.start_subtest("F_ALT_MIN zero - disabled, can't arm in face of threat")
        self.set_parameters({
            "AVD_F_ALT_MIN": 0,
        })
        self.wait_ready_to_arm()
        self.test_adsb_send_threatening_adsb_message(here)
        self.delay_sim_time(1, reason="ADSB threat to be detected")
        self.try_arm(result=False,
                     expect_msg="ADSB threat detected")

        self.wait_ready_to_arm(timeout=60)

        self.context_pop()

        self.start_subtest("F_ALT_MIN 16m relative - arm in face of threat")
        self.context_push()
        self.set_parameters({
            "AVD_F_ALT_MIN": int(16 + here.get_alt_m(AltFrame.ABSOLUTE)),
        })
        self.wait_ready_to_arm()
        self.test_adsb_send_threatening_adsb_message(here)
#        self.delay_sim_time(1, "let data in log accrue")
        self.arm_vehicle()
        self.disarm_vehicle()
        self.context_pop()
