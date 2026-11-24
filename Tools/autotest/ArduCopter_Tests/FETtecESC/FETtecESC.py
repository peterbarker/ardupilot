'''
Copter autotest: FETtecESC

AP_FLAKE8_CLEAN
'''

from vehicle_test_suite import NotAchievedException

from test_discovery import TestGroup
from test_discovery import testcase


class FETtecESC(TestGroup):
    def FETtecESC_flight(self):
        '''fly with servo outputs from FETtec ESC'''
        self.start_subtest("FETtec ESC flight")
        num_wp = self.load_mission("copter_mission.txt", strict=False)
        self.fly_loaded_mission(num_wp)

    def FETtecESC_esc_power_checks(self):
        '''Make sure state machine copes with ESCs rebooting'''
        self.start_subtest("FETtec ESC reboot")
        self.wait_ready_to_arm()
        self.context_collect('STATUSTEXT')
        self.progress("Turning off an ESC off ")
        mask = int(self.get_parameter("SIM_FTOWESC_POW"))

        for mot_id_to_kill in 1, 2:
            self.progress("Turning ESC=%u off" % mot_id_to_kill)
            self.set_parameter("SIM_FTOWESC_POW", mask & ~(1 << mot_id_to_kill))
            self.delay_sim_time(1, reason="ESC power-off to be detected")
            self.assert_prearm_failure("are not running")
            self.progress("Turning it back on")
            self.set_parameter("SIM_FTOWESC_POW", mask)
            self.wait_ready_to_arm()

            self.progress("Turning ESC=%u off (again)" % mot_id_to_kill)
            self.set_parameter("SIM_FTOWESC_POW", mask & ~(1 << mot_id_to_kill))
            self.delay_sim_time(1, reason="ESC power-off to be detected")
            self.assert_prearm_failure("are not running")
            self.progress("Turning it back on")
            self.set_parameter("SIM_FTOWESC_POW", mask)
            self.wait_ready_to_arm()

        self.progress("Turning all ESCs off")
        self.set_parameter("SIM_FTOWESC_POW", 0)
        self.delay_sim_time(1, reason="ESC power-off to be detected")
        self.assert_prearm_failure("are not running")
        self.progress("Turning them back on")
        self.set_parameter("SIM_FTOWESC_POW", mask)
        self.wait_ready_to_arm()

    def fettec_assert_bad_mask(self, mask):
        '''assert the mask is bad for fettec driver'''
        self.start_subsubtest("Checking mask (%s) is bad" % (mask,))
        self.context_push()
        self.set_parameter("SERVO_FTW_MASK", mask)
        self.reboot_sitl()
        self.delay_sim_time(12, reason="accels/gyros to be happy")  # allow accels/gyros to be happy
        tstart = self.get_sim_time()
        while True:
            if self.get_sim_time_cached() - tstart > 20:
                raise NotAchievedException("Expected mask to be only problem within 20 seconds")
            try:
                self.assert_prearm_failure("Invalid motor mask")
                break
            except NotAchievedException:
                self.delay_sim_time(1, reason="poll interval")
        self.context_pop()
        self.reboot_sitl()

    def fettec_assert_good_mask(self, mask):
        '''assert the mask is bad for fettec driver'''
        self.start_subsubtest("Checking mask (%s) is good" % (mask,))
        self.context_push()
        self.set_parameter("SERVO_FTW_MASK", mask)
        self.reboot_sitl()
        self.wait_ready_to_arm()
        self.context_pop()
        self.reboot_sitl()

    def FETtecESC_safety_switch(self):
        mot = self.find_first_set_bit(int(self.get_parameter("SERVO_FTW_MASK"))) + 1
        self.wait_esc_telem_rpm(mot, 0, 0)
        self.wait_ready_to_arm()
        self.context_push()
        self.set_parameter("DISARM_DELAY", 0)
        self.arm_vehicle()
        # we have to wait for a while for the arming tone to go out
        # before the motors will spin:
        self.wait_esc_telem_rpm(
            esc=mot,
            rpm_min=17640,
            rpm_max=17640,
            minimum_duration=2,
            timeout=5,
        )
        self.set_safetyswitch_on()
        self.wait_esc_telem_rpm(mot, 0, 0)
        self.set_safetyswitch_off()
        self.wait_esc_telem_rpm(
            esc=mot,
            rpm_min=17640,
            rpm_max=17640,
            minimum_duration=2,
            timeout=5,
        )
        self.context_pop()
        self.wait_disarmed()

    def FETtecESC_btw_mask_checks(self):
        '''ensure prearm checks work as expected'''
        for bad_mask in [0b1000000000000000, 0b10100000000000000]:
            self.fettec_assert_bad_mask(bad_mask)
        for good_mask in [0b00001, 0b00101, 0b110000000000]:
            self.fettec_assert_good_mask(good_mask)

    @testcase(bucket="CopterTests1d")
    def FETtecESC(self):
        '''Test FETtecESC'''
        self.set_parameters({
            "SERIAL5_PROTOCOL": 38,
            "SERVO_FTW_MASK": 0b11101000,
            "SIM_FTOWESC_ENA": 1,
            "SERVO1_FUNCTION": 0,
            "SERVO2_FUNCTION": 0,
            "SERVO3_FUNCTION": 0,
            "SERVO4_FUNCTION": 33,
            "SERVO5_FUNCTION": 0,
            "SERVO6_FUNCTION": 34,
            "SERVO7_FUNCTION": 35,
            "SERVO8_FUNCTION": 36,
            "SIM_ESC_TELEM": 0,
        })
        self.customise_SITL_commandline(["--serial5=sim:fetteconewireesc"])
        self.FETtecESC_safety_switch()
        self.FETtecESC_esc_power_checks()
        self.FETtecESC_btw_mask_checks()
        self.FETtecESC_flight()
