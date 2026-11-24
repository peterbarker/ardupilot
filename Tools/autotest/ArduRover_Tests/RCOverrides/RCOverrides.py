'''
Rover autotest: RCOverrides

AP_FLAKE8_CLEAN
'''

from vehicle_test_suite import AutoTestTimeoutException
from vehicle_test_suite import NotAchievedException

from test_discovery import TestGroup
from test_discovery import testcase


class RCOverrides(TestGroup):
    @testcase(bucket="Rover")
    def RCOverrides(self):
        '''Test RC overrides'''
        self.set_parameter("MAV_GCS_SYSID", self.mav.source_system)
        self.set_parameter("RC12_OPTION", 46)
        self.reboot_sitl()

        self.change_mode('MANUAL')
        self.wait_ready_to_arm()
        self.set_rc(3, 1500)  # throttle at zero
        self.arm_vehicle()
        # start moving forward a little:
        normal_rc_throttle = 1700
        self.set_rc(3, normal_rc_throttle)
        self.wait_groundspeed(5, 100)

        # allow overrides:
        self.set_rc(12, 2000)

        # now override to stop:
        throttle_override = 1500

        tstart = self.get_sim_time_cached()
        while True:
            if self.get_sim_time_cached() - tstart > 10:
                raise AutoTestTimeoutException("Did not reach speed")
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

            m = self.assert_receive_message('VFR_HUD')
            want_speed = 2.0
            self.progress("Speed=%f want=<%f" % (m.groundspeed, want_speed))
            if m.groundspeed < want_speed:
                break

        # now override to stop - but set the switch on the RC
        # transmitter to deny overrides; this should send the
        # speed back up to 5 metres/second:
        self.set_rc(12, 1000)

        throttle_override = 1500
        tstart = self.get_sim_time_cached()
        while True:
            if self.get_sim_time_cached() - tstart > 10:
                raise AutoTestTimeoutException("Did not speed back up")
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

            m = self.assert_receive_message('VFR_HUD')
            want_speed = 5.0
            self.progress("Speed=%f want=>%f" % (m.groundspeed, want_speed))

            if m.groundspeed > want_speed:
                break

        # re-enable RC overrides
        self.set_rc(12, 2000)

        # check we revert to normal RC inputs when gcs overrides cease:
        self.progress("Waiting for RC to revert to normal RC input")
        self.wait_rc_channel_value(3, normal_rc_throttle, timeout=10)

        self.start_subtest("Check override time of zero disables overrides")
        old = self.get_parameter("RC_OVERRIDE_TIME")
        ch = 2
        self.set_rc(ch, 1000)
        channels = [65535] * 18
        ch_override_value = 1700
        channels[ch-1] = ch_override_value
        channels[7] = 1234 # that's channel 8!
        self.progress("Sending override message %u" % ch_override_value)
        self.mav.mav.rc_channels_override_send(
            1, # target system
            1, # targe component
            *channels
        )
        # long timeout required here as we may have sent a lot of
        # things via MAVProxy...
        self.wait_rc_channel_value(ch, ch_override_value, timeout=30)
        self.set_parameter("RC_OVERRIDE_TIME", 0)
        self.wait_rc_channel_value(ch, 1000)
        self.set_parameter("RC_OVERRIDE_TIME", old)
        # an override is only live for RC_OVERRIDE_TIME seconds after it
        # arrives, and the two parameter round-trips above can eat more
        # than that in simulated time - one run came back to find the
        # override long expired, and sat watching chan2 stay at 1000.
        # Send a fresh one, which is what re-enabling overrides has to
        # act upon anyway.
        self.progress("Sending override message %u" % ch_override_value)
        self.mav.mav.rc_channels_override_send(
            1, # target system
            1, # targe component
            *channels
        )
        self.wait_rc_channel_value(ch, ch_override_value)

        ch_override_value = 1720
        channels[ch-1] = ch_override_value
        self.progress("Sending override message %u" % ch_override_value)
        self.mav.mav.rc_channels_override_send(
            1, # target system
            1, # targe component
            *channels
        )
        self.wait_rc_channel_value(ch, ch_override_value, timeout=10)
        self.set_parameter("RC_OVERRIDE_TIME", 0)
        self.wait_rc_channel_value(ch, 1000)
        self.set_parameter("RC_OVERRIDE_TIME", old)

        self.progress("Ensuring timeout works")
        self.wait_rc_channel_value(ch, 1000, timeout=5)
        self.delay_sim_time(10, reason="RC override timeout")

        self.set_parameter("RC_OVERRIDE_TIME", 10)
        self.progress("Sending override message")

        ch_override_value = 1730
        channels[ch-1] = ch_override_value
        self.progress("Sending override message %u" % ch_override_value)
        self.mav.mav.rc_channels_override_send(
            1, # target system
            1, # targe component
            *channels
        )
        self.wait_rc_channel_value(ch, ch_override_value, timeout=10)
        tstart = self.get_sim_time()
        self.progress("Waiting for channel to revert to 1000 in ~10s")
        self.wait_rc_channel_value(ch, 1000, timeout=15)
        delta = self.get_sim_time() - tstart
        if delta > 12:
            raise NotAchievedException("Took too long to revert RC channel value (delta=%f)" % delta)
        min_delta = 9
        if delta < min_delta:
            raise NotAchievedException("Didn't take long enough to revert RC channel value (delta=%f want>=%f)" %
                                       (delta, min_delta))
        self.progress("Disabling RC override timeout")
        self.set_parameter("RC_OVERRIDE_TIME", -1)
        ch_override_value = 1740
        channels[ch-1] = ch_override_value
        self.progress("Sending override message %u" % ch_override_value)
        self.mav.mav.rc_channels_override_send(
            1, # target system
            1, # targe component
            *channels
        )
        self.wait_rc_channel_value(ch, ch_override_value, timeout=10)
        tstart = self.get_sim_time()
        while True:
            # warning: this is get_sim_time() and can slurp messages on you!
            delta = self.get_sim_time() - tstart
            if delta > 20:
                break
            m = self.assert_receive_message('RC_CHANNELS')
            channel_field = "chan%u_raw" % ch
            m_value = getattr(m, channel_field)
            if m_value != ch_override_value:
                raise NotAchievedException("Value reverted after %f seconds when it should not have (got=%u) (want=%u)" % (delta, m_value, ch_override_value))  # noqa
        self.set_parameter("RC_OVERRIDE_TIME", old)

        self.wait_rc_channel_value(ch, 1000, timeout=15)

        self.start_subtest("Checking higher-channel semantics")
        self.context_push()
        self.set_parameter("RC_OVERRIDE_TIME", 30)

        ch = 11
        rc_value = 1010
        self.set_rc(ch, rc_value)

        channels = [65535] * 18
        ch_override_value = 1234
        channels[ch-1] = ch_override_value
        self.progress("Sending override message ch%u=%u" % (ch, ch_override_value))
        self.mav.mav.rc_channels_override_send(
            1, # target system
            1, # targe component
            *channels
        )
        self.progress("Wait for override value")
        self.wait_rc_channel_value(ch, ch_override_value, timeout=10)

        self.progress("Sending return-to-RC-input value")
        channels[ch-1] = 65534
        self.mav.mav.rc_channels_override_send(
            1, # target system
            1, # targe component
            *channels
        )
        self.wait_rc_channel_value(ch, rc_value, timeout=10)

        channels[ch-1] = ch_override_value
        self.progress("Sending override message ch%u=%u" % (ch, ch_override_value))
        self.mav.mav.rc_channels_override_send(
            1, # target system
            1, # targe component
            *channels
        )
        self.progress("Wait for override value")
        self.wait_rc_channel_value(ch, ch_override_value, timeout=10)

        # make we keep the override value for at least 10 seconds:
        tstart = self.get_sim_time()
        while True:
            if self.get_sim_time_cached() - tstart > 10:
                break
            # try both ignore values:
            ignore_value = 0
            if self.get_sim_time_cached() - tstart > 5:
                ignore_value = 65535
            self.progress("Sending ignore value %u" % ignore_value)
            channels[ch-1] = ignore_value
            self.mav.mav.rc_channels_override_send(
                1, # target system
                1, # targe component
                *channels
            )
            if self.get_rc_channel_value(ch) != ch_override_value:
                raise NotAchievedException("Did not maintain value")

        self.context_pop()

        self.end_subtest("Checking higher-channel semantics")

        self.disarm_vehicle()
