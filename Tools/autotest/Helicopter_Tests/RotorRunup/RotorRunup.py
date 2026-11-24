'''
Helicopter autotest: RotorRunup

AP_FLAKE8_CLEAN
'''

from vehicle_test_suite import NotAchievedException

from test_discovery import TestGroup
from test_discovery import testcase


class RotorRunup(TestGroup):
    @testcase(bucket="Helicopter")
    def RotorRunup(self):
        '''Test rotor runup'''
        # Takeoff and landing in Loiter
        TARGET_RUNUP_TIME = 10
        self.change_mode('LOITER')
        self.wait_ready_to_arm()
        self.arm_vehicle()
        servo = self.assert_receive_message('SERVO_OUTPUT_RAW')
        coll = servo.servo1_raw
        coll = coll + 50
        self.set_parameter("H_RSC_RUNUP_TIME", TARGET_RUNUP_TIME)
        self.progress("Initiate Runup by putting some throttle")
        tstart = self.get_sim_time()
        self.set_rc_from_map({8: 2000, 3: 1700})
        self.progress("Collective threshold PWM %u" % coll)
        self.progress("Wait that collective PWM pass threshold value")
        servo = self.assert_receive_message(
            "SERVO_OUTPUT_RAW",
            condition=f'SERVO_OUTPUT_RAW.servo1_raw>{coll}'
        )
        runup_time = self.get_sim_time() - tstart
        self.progress("Collective is now at PWM %u" % servo.servo1_raw)
        if runup_time < TARGET_RUNUP_TIME:
            raise NotAchievedException("Takeoff initiated before runup time complete %u" % runup_time)
        self.progress("Runup time %u" % runup_time)
        self.zero_throttle()
        self.land_and_disarm()
