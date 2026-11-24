'''
Helicopter autotest: DDFPTail

AP_FLAKE8_CLEAN
'''

from vehicle_test_suite import NotAchievedException

from test_discovery import TestGroup
from test_discovery import testcase


class DDFPTail(TestGroup):
    @testcase(bucket="Helicopter")
    def DDFPTail(self):
        # simple check to ensure servo output to DDFP tail is working
        '''Test DDFP Tail Rotor'''
        self.customise_SITL_commandline(
            [],
            defaults_filepath=self.model_defaults_filepath('heli-ddfptail'),
            model="heli-ddfptail",
            wipe=True,
        )
        self.takeoff(10)
        self.wait_servo_channel_in_range(7, 1402, 1406, timeout=10)
        self.do_RTL()
        # check that battery compensation logic
        self.set_parameters({
            "H_DDFP_BAT_V_MAX": 50.4,
            "H_DDFP_BAT_V_MIN": 39.6,
            "SIM_BATT_CAP_AH": 5.0,
            "GCS_PID_MASK": 4.0,
            "ATC_RAT_YAW_I": 0.00001
        })
        self.takeoff(10)
        self.change_mode("LOITER")
        self.delay_sim_time(500, reason="allow time for battery to deplete and battery compensation to take effect")
        m = self.assert_receive_message('PID_TUNING')
        if m.P > 0.42:
            raise NotAchievedException("Battery compensation not working")
        self.set_parameter("ATC_RAT_YAW_I", 0.2)
        self.do_RTL()
