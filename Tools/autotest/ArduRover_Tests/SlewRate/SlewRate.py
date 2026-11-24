'''
Rover autotest: SlewRate

AP_FLAKE8_CLEAN
'''

from vehicle_test_suite import NotAchievedException

from test_discovery import TestGroup
from test_discovery import testcase


class SlewRate(TestGroup):
    @testcase(bucket="Rover", disabled="got timing report failure on CI")
    def SlewRate(self):
        """Test Motor Slew Rate feature."""
        self.context_push()
        self.change_mode("MANUAL")
        self.wait_ready_to_arm()
        self.arm_vehicle()

        self.start_subtest("Test no slew behavior")
        throttle_channel = 3
        throttle_max = 2000
        self.set_parameter("MOT_SLEWRATE", 0)
        self.set_rc(throttle_channel, throttle_max)
        tstart = self.get_sim_time()
        self.wait_servo_channel_value(throttle_channel, throttle_max)
        tstop = self.get_sim_time()
        achieved_time = tstop - tstart
        self.progress("achieved_time: %0.1fs" % achieved_time)
        if achieved_time > 0.5:
            raise NotAchievedException("Output response should be instant, got %f" % achieved_time)
        self.zero_throttle()
        self.wait_groundspeed(0, 0.5)  # why do we not stop?!

        self.start_subtest("Test 100% slew rate")
        self.set_parameter("MOT_SLEWRATE", 100)
        self.set_rc(throttle_channel, throttle_max)
        tstart = self.get_sim_time()
        self.wait_servo_channel_value(throttle_channel, throttle_max)
        tstop = self.get_sim_time()
        achieved_time = tstop - tstart
        self.progress("achieved_time: %0.1fs" % achieved_time)
        if achieved_time < 0.9 or achieved_time > 1.1:
            raise NotAchievedException("Output response should be 1s, got %f" % achieved_time)
        self.zero_throttle()
        self.wait_groundspeed(0, 0.5)  # why do we not stop?!

        self.start_subtest("Test 50% slew rate")
        self.set_parameter("MOT_SLEWRATE", 50)
        self.set_rc(throttle_channel, throttle_max)
        tstart = self.get_sim_time()
        self.wait_servo_channel_value(throttle_channel, throttle_max, timeout=10)
        tstop = self.get_sim_time()
        achieved_time = tstop - tstart
        self.progress("achieved_time: %0.1fs" % achieved_time)
        if achieved_time < 1.8 or achieved_time > 2.2:
            raise NotAchievedException("Output response should be 2s, got %f" % achieved_time)
        self.zero_throttle()
        self.wait_groundspeed(0, 0.5)  # why do we not stop?!

        self.start_subtest("Test 25% slew rate")
        self.set_parameter("MOT_SLEWRATE", 25)
        self.set_rc(throttle_channel, throttle_max)
        tstart = self.get_sim_time()
        self.wait_servo_channel_value(throttle_channel, throttle_max, timeout=10)
        tstop = self.get_sim_time()
        achieved_time = tstop - tstart
        self.progress("achieved_time: %0.1fs" % achieved_time)
        if achieved_time < 3.6 or achieved_time > 4.4:
            raise NotAchievedException("Output response should be 4s, got %f" % achieved_time)
        self.zero_throttle()
        self.wait_groundspeed(0, 0.5)  # why do we not stop?!

        self.start_subtest("Test 10% slew rate")
        self.set_parameter("MOT_SLEWRATE", 10)
        self.set_rc(throttle_channel, throttle_max)
        tstart = self.get_sim_time()
        self.wait_servo_channel_value(throttle_channel, throttle_max, timeout=20)
        tstop = self.get_sim_time()
        achieved_time = tstop - tstart
        self.progress("achieved_time: %0.1fs" % achieved_time)
        if achieved_time < 9 or achieved_time > 11:
            raise NotAchievedException("Output response should be 10s, got %f" % achieved_time)
        self.zero_throttle()
        self.wait_groundspeed(0, 0.5)  # why do we not stop?!
        self.disarm_vehicle()
        self.context_pop()
