'''
Plane autotest: ThrottleFailsafe

AP_FLAKE8_CLEAN
'''

from pymavlink import mavutil

from vehicle_test_suite import NotAchievedException
from vehicle_test_suite import PreconditionFailedException

from test_discovery import TestGroup
from test_discovery import testcase


class ThrottleFailsafe(TestGroup):
    @testcase(bucket="PlaneTests1b")
    def ThrottleFailsafe(self):
        '''Fly throttle failsafe'''
        self.change_mode('MANUAL')
        m = self.assert_receive_message('SYS_STATUS')
        receiver_bit = mavutil.mavlink.MAV_SYS_STATUS_SENSOR_RC_RECEIVER
        self.progress("Testing receiver enabled")
        if (not (m.onboard_control_sensors_enabled & receiver_bit)):
            raise PreconditionFailedException()
        self.progress("Testing receiver present")
        if (not (m.onboard_control_sensors_present & receiver_bit)):
            raise PreconditionFailedException()
        self.progress("Testing receiver health")
        if (not (m.onboard_control_sensors_health & receiver_bit)):
            raise PreconditionFailedException()

        self.progress("Ensure we know original throttle value")
        self.wait_rc_channel_value(3, 1000)

        self.set_parameter("THR_FS_VALUE", 960)
        self.progress("Failing receiver (throttle-to-950)")
        self.context_collect("HEARTBEAT")
        self.set_parameter("SIM_RC_FAIL", 2) # throttle-to-950
        self.wait_mode('RTL') # long failsafe
        if (self.get_mode_from_mode_mapping("CIRCLE") not in
                [x.custom_mode for x in self.context_stop_collecting("HEARTBEAT")]):
            raise NotAchievedException("Did not go via circle mode")
        self.progress("Ensure we've had our throttle squashed to 950")
        self.wait_rc_channel_value(3, 950)
        self.do_timesync_roundtrip()
        m = self.assert_receive_message('SYS_STATUS')
        self.progress("Got (%s)" % str(m))
        self.progress("Testing receiver enabled")
        if (not (m.onboard_control_sensors_enabled & receiver_bit)):
            raise NotAchievedException("Receiver not enabled")
        self.progress("Testing receiver present")
        if (not (m.onboard_control_sensors_present & receiver_bit)):
            raise NotAchievedException("Receiver not present")
        # skip this until RC is fixed
#        self.progress("Testing receiver health")
#        if (m.onboard_control_sensors_health & receiver_bit):
#            raise NotAchievedException("Sensor healthy when it shouldn't be")
        self.set_parameter("SIM_RC_FAIL", 0)
        # wait for RC to be fetched from SITL and the receiver to recover
        self.wait_sensor_state(receiver_bit, True, True, True)
        self.do_timesync_roundtrip()
        m = self.assert_receive_message('SYS_STATUS')
        self.progress("Testing receiver enabled")
        if (not (m.onboard_control_sensors_enabled & receiver_bit)):
            raise NotAchievedException("Receiver not enabled")
        self.progress("Testing receiver present")
        if (not (m.onboard_control_sensors_present & receiver_bit)):
            raise NotAchievedException("Receiver not present")
        self.progress("Testing receiver health")
        if (not (m.onboard_control_sensors_health & receiver_bit)):
            raise NotAchievedException("Receiver not healthy2")
        self.change_mode('MANUAL')

        self.progress("Failing receiver (no-pulses)")
        self.context_collect("HEARTBEAT")
        self.set_parameter("SIM_RC_FAIL", 1) # no-pulses
        self.wait_mode('RTL') # long failsafe
        if (self.get_mode_from_mode_mapping("CIRCLE") not in
                [x.custom_mode for x in self.context_stop_collecting("HEARTBEAT")]):
            raise NotAchievedException("Did not go via circle mode")
        self.do_timesync_roundtrip()
        m = self.assert_receive_message('SYS_STATUS')
        self.progress("Got (%s)" % str(m))
        self.progress("Testing receiver enabled")
        if (not (m.onboard_control_sensors_enabled & receiver_bit)):
            raise NotAchievedException("Receiver not enabled")
        self.progress("Testing receiver present")
        if (not (m.onboard_control_sensors_present & receiver_bit)):
            raise NotAchievedException("Receiver not present")
        self.progress("Testing receiver health")
        if (m.onboard_control_sensors_health & receiver_bit):
            raise NotAchievedException("Sensor healthy when it shouldn't be")
        self.progress("Making RC work again")
        self.set_parameter("SIM_RC_FAIL", 0)
        self.progress("Giving receiver time to recover")
        self.wait_sensor_state(receiver_bit, True, True, True)
        self.do_timesync_roundtrip()
        m = self.assert_receive_message('SYS_STATUS')
        self.progress("Testing receiver enabled")
        if (not (m.onboard_control_sensors_enabled & receiver_bit)):
            raise NotAchievedException("Receiver not enabled")
        self.progress("Testing receiver present")
        if (not (m.onboard_control_sensors_present & receiver_bit)):
            raise NotAchievedException("Receiver not present")
        self.progress("Testing receiver health")
        if (not (m.onboard_control_sensors_health & receiver_bit)):
            raise NotAchievedException("Receiver not healthy")
        self.change_mode('MANUAL')

        self.progress("Ensure long failsafe can trigger when short failsafe disabled")
        self.context_push()
        self.context_collect("STATUSTEXT")
        self.set_parameters({
            "FS_SHORT_ACTN": 3, # 3 means disabled
            "SIM_RC_FAIL": 1,
        })
        self.wait_statustext("Long failsafe on", check_context=True)
        self.wait_mode("RTL")
#            self.context_clear_collection("STATUSTEXT")
        self.set_parameter("SIM_RC_FAIL", 0)
        self.wait_text("Long Failsafe Cleared", check_context=True)
        self.change_mode("MANUAL")

        self.progress("Trying again with THR_FS_VALUE")
        self.set_parameters({
            "THR_FS_VALUE": 960,
            "SIM_RC_FAIL": 2,
        })
        self.wait_statustext("Long Failsafe on", check_context=True)
        self.wait_mode("RTL")
        self.context_pop()

        self.start_subtest("Not use RC throttle input when THR_FAILSAFE==2")
        self.takeoff(100)
        self.set_rc_from_map({3: 1800, 1: 2000})
        self.wait_attitude(desroll=45, timeout=1)
        self.context_push()
        self.set_parameters({
            "THR_FAILSAFE": 2,
            "SIM_RC_FAIL": 1,  # no pulses
        })
        self.delay_sim_time(1, reason="failsafe to take effect")
        self.wait_attitude(desroll=0, timeout=5)
        self.assert_servo_channel_value(3, self.get_parameter("RC3_MIN"))
        self.set_parameters({
            "SIM_RC_FAIL": 0,  # fix receiver
        })
        self.zero_throttle()
        self.disarm_vehicle(force=True)
        self.context_pop()
        self.reboot_sitl()
