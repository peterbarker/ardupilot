'''
Rover autotest: RCOverridesCancel

AP_FLAKE8_CLEAN
'''

from vehicle_test_suite import AutoTestTimeoutException

from test_discovery import TestGroup
from test_discovery import testcase


class RCOverridesCancel(TestGroup):
    @testcase(bucket="Rover")
    def RCOverridesCancel(self):
        '''Test RC overrides Cancel'''
        self.set_parameter("MAV_GCS_SYSID", self.mav.source_system)
        self.change_mode('MANUAL')
        self.wait_ready_to_arm()
        self.zero_throttle()
        self.arm_vehicle()
        # start moving forward a little:
        normal_rc_throttle = 1700
        throttle_override = 1900

        self.progress("Establishing baseline RC input")
        self.set_rc(3, normal_rc_throttle)
        self.drain_mav()
        tstart = self.get_sim_time()
        while True:
            if self.get_sim_time_cached() - tstart > 10:
                raise AutoTestTimeoutException("Did not get rc change")
            m = self.assert_receive_message('RC_CHANNELS')
            if m.chan3_raw == normal_rc_throttle:
                break

        self.progress("Set override with RC_CHANNELS_OVERRIDE")
        self.drain_mav()
        tstart = self.get_sim_time()
        while True:
            if self.get_sim_time_cached() - tstart > 10:
                raise AutoTestTimeoutException("Did not override")
            self.progress("Sending throttle of %u" % (throttle_override,))
            self.mav.mav.rc_channels_override_send(
                1, # target system
                1, # targe component
                65535, # chan1_raw
                65535, # chan2_raw
                throttle_override, # chan3_raw
                65535, # chan4_raw
                65535, # chan5_raw
                65535, # chan6_raw
                65535, # chan7_raw
                65535) # chan8_raw

            m = self.assert_receive_message('RC_CHANNELS')
            self.progress("chan3=%f want=%f" % (m.chan3_raw, throttle_override))
            if m.chan3_raw == throttle_override:
                break

        self.progress("disabling override and making sure we revert to RC input in good time")
        self.drain_mav()
        tstart = self.get_sim_time()
        while True:
            if self.get_sim_time_cached() - tstart > 0.5:
                raise AutoTestTimeoutException("Did not cancel override")
            self.progress("Sending cancel of throttle override")
            self.mav.mav.rc_channels_override_send(
                1, # target system
                1, # targe component
                65535, # chan1_raw
                65535, # chan2_raw
                0,     # chan3_raw
                65535, # chan4_raw
                65535, # chan5_raw
                65535, # chan6_raw
                65535, # chan7_raw
                65535) # chan8_raw
            self.do_timesync_roundtrip()
            m = self.assert_receive_message('RC_CHANNELS')
            self.progress("chan3=%f want=%f" % (m.chan3_raw, normal_rc_throttle))
            if m.chan3_raw == normal_rc_throttle:
                break
        self.disarm_vehicle()
