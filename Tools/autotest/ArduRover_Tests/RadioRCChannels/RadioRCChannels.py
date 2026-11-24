'''
Rover autotest: RadioRCChannels

AP_FLAKE8_CLEAN
'''

from pymavlink import mavutil

from vehicle_test_suite import NotAchievedException

from test_discovery import TestGroup
from test_discovery import testcase


class RadioRCChannels(TestGroup):
    @testcase(bucket="Rover", speedup=5)  # we hand-feed the RC frames
    def RadioRCChannels(self):
        '''test RC input supplied via the RADIO_RC_CHANNELS mavlink message'''

        # channel values are in the message's centered 13-bit format;
        # [-4096,4096] maps onto [860,2140] PWM.  Channel 8 is left where the
        # test suite's default RC puts it so the mode switch does not move:
        values = [
            0,      # 1500
            -4096,  # 860; minimum
            4096,   # 2140; maximum
            1600,   # 1750
            -1600,  # 1250
            100,    # 1515; division truncates towards zero
            -100,   # 1485; ... in both directions
            1920,   # 1800; mode channel
            320,    # 1550
            -320,   # 1450
            800,    # 1625
            -800,   # 1375
            2400,   # 1875
            -2400,  # 1125
            3200,   # 2000
            -3200,  # 1000
        ]
        expected_pwm = [self.radio_rc_channels_value_to_pwm(x) for x in values]

        self.set_parameters({
            "RC_PROTOCOLS": 1 << 16,  # MAVRadio only; excludes the SITL UDP RC input
            "RC_FS_TIMEOUT": 3,  # we feed the frames by hand; be generous
            "FS_ACTION": 0,  # we want the failsafe reported, not acted upon
        })
        # RC_PROTOCOLS is only consulted while AP_RCProtocol is searching for
        # a protocol, and the SITL UDP input has long since been detected:
        self.context_collect('STATUSTEXT')
        self.reboot_sitl()

        self.start_subtest("no RC input at all until a RADIO_RC_CHANNELS arrives")
        self.delay_sim_time(5, reason="let any RC input appear")
        self.assert_sensor_state(
            mavutil.mavlink.MAV_SYS_STATUS_SENSOR_RC_RECEIVER,
            present=False,
            enabled=False,
            healthy=False,
        )
        m = self.assert_receive_message('RC_CHANNELS')
        if m.chancount != 0:
            raise NotAchievedException("Expected no RC channels, got %u" % m.chancount)

        self.start_subtest("channel values are converted to PWM")
        self.wait_radio_rc_channels_pwm(values, expected_pwm, len(values))
        self.wait_statustext("RCInput: decoding MAVRadio", check_context=True, timeout=10)
        self.context_stop_collecting('STATUSTEXT')

        self.start_subtest("every frame is converted, not just the first")
        # the same count, but a different value in every channel bar the mode
        # channel; a backend which converted the first frame it saw and then
        # latched would still be reporting the PWMs from the subtest above.
        # All of the values are distinct, so rotating them moves every one:
        moved = [x for (i, x) in enumerate(values) if i != 7]
        moved = moved[1:] + moved[0:1]
        moved.insert(7, values[7])
        moved_pwm = [self.radio_rc_channels_value_to_pwm(x) for x in moved]
        self.wait_radio_rc_channels_pwm(moved, moved_pwm, len(moved))

        self.start_subtest("count is honoured")
        # the payload always carries 32 channels, so count is the only thing
        # saying how many of them are real; send all sixteen values but claim
        # only eight, so honouring count and honouring the payload differ:
        self.wait_radio_rc_channels_pwm(values, expected_pwm[0:8], 8, count=8)

        self.start_subtest("oversize count is clamped")
        # the message carries 32 channels and ArduPilot takes at most
        # MAX_RCIN_CHANNELS of them.  RC_CHANNELS is all we can see here and
        # it reports at most NUM_RC_CHANNELS, so this demonstrates only that
        # no more than 16 channels come back, not where the input-side clamp
        # sits:
        self.wait_radio_rc_channels_pwm(values + values, expected_pwm, 16)

        self.start_subtest("failsafe flag is honoured")
        self.set_parameter("FS_THR_ENABLE", 1)
        # establish that the vehicle is happy while the frames flow:
        self.radio_rc_channels_pump(
            values,
            0,
            'SYS_STATUS',
            lambda m: m.onboard_control_sensors_health & mavutil.mavlink.MAV_SYS_STATUS_SENSOR_RC_RECEIVER,
            20,
            "RC receiver to be healthy",
        )
        # ... then change nothing but the failsafe flag:
        self.context_collect('STATUSTEXT')
        self.radio_rc_channels_pump(
            values,
            mavutil.mavlink.RADIO_RC_CHANNELS_FLAGS_FAILSAFE,
            'SYS_STATUS',
            lambda m: not (m.onboard_control_sensors_health & mavutil.mavlink.MAV_SYS_STATUS_SENSOR_RC_RECEIVER),
            30,
            "RC receiver to go unhealthy",
        )
        # note that it is the SYS_STATUS assertion above - made while the
        # frames were still flowing - which shows the flag caused this.  The
        # statustext wait below runs after the pump has stopped, so on its own
        # it would also be satisfied by simply losing the input:
        self.wait_statustext("Radio Failsafe", check_context=True, timeout=30)
        self.context_stop_collecting('STATUSTEXT')

        self.start_subtest("failsafe clears when the flag clears")
        self.context_collect('STATUSTEXT')
        self.radio_rc_channels_pump(
            values,
            0,
            'SYS_STATUS',
            lambda m: m.onboard_control_sensors_health & mavutil.mavlink.MAV_SYS_STATUS_SENSOR_RC_RECEIVER,
            30,
            "RC receiver to recover",
        )
        self.wait_statustext("Radio Failsafe Cleared", check_context=True, timeout=30)
        self.context_stop_collecting('STATUSTEXT')

        self.start_subtest("a flag which is not FAILSAFE does not fail the vehicle")
        # a backend testing bool(flags) rather than the FAILSAFE bit would put
        # us into failsafe here.  Maintain for longer than RC_FS_TIMEOUT, so
        # input which had stopped counting as valid would show up:
        self.assert_radio_rc_channels_maintains(
            values,
            mavutil.mavlink.RADIO_RC_CHANNELS_FLAGS_OUTDATED,
            'SYS_STATUS',
            lambda m: m.onboard_control_sensors_health & mavutil.mavlink.MAV_SYS_STATUS_SENSOR_RC_RECEIVER,
            6,  # RC_FS_TIMEOUT is 3
            "RC receiver went unhealthy with only the OUTDATED flag set",
        )

        self.start_subtest("FAILSAFE is honoured when another flag is also set")
        # ... while a backend comparing flags with == rather than masking
        # would ignore FAILSAFE the moment any other bit came along, so send
        # the pair the earlier subtests have shown to be individually inert
        # and active:
        self.radio_rc_channels_pump(
            values,
            (mavutil.mavlink.RADIO_RC_CHANNELS_FLAGS_FAILSAFE |
             mavutil.mavlink.RADIO_RC_CHANNELS_FLAGS_OUTDATED),
            'SYS_STATUS',
            lambda m: not (m.onboard_control_sensors_health & mavutil.mavlink.MAV_SYS_STATUS_SENSOR_RC_RECEIVER),
            30,
            "RC receiver to go unhealthy with FAILSAFE and OUTDATED both set",
        )

        self.start_subtest("RADIO_RC_CHANNELS is ignored when RC_PROTOCOLS excludes MAVRadio")
        # PPM only, which also excludes the SITL UDP RC input, so nothing at
        # all should ever count as RC input; the same frames as were accepted
        # above must not make the RC receiver appear:
        self.set_parameter("RC_PROTOCOLS", 1 << 1)
        self.reboot_sitl()
        self.assert_radio_rc_channels_maintains(
            values,
            0,
            'SYS_STATUS',
            lambda m: not (m.onboard_control_sensors_present & mavutil.mavlink.MAV_SYS_STATUS_SENSOR_RC_RECEIVER),
            10,
            "RC receiver appeared with MAVRadio excluded from RC_PROTOCOLS",
        )
        m = self.assert_receive_message('RC_CHANNELS')
        if m.chancount != 0:
            raise NotAchievedException("Expected no RC channels, got %u" % m.chancount)
