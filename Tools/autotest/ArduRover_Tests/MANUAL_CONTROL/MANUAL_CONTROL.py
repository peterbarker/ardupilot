'''
Rover autotest: MANUAL_CONTROL

AP_FLAKE8_CLEAN
'''

from vehicle_test_suite import AutoTestTimeoutException

from test_discovery import TestGroup
from test_discovery import testcase


class MANUAL_CONTROL(TestGroup):
    @testcase(bucket="Rover")
    def MANUAL_CONTROL(self):
        '''Test mavlink MANUAL_CONTROL'''
        self.set_parameters({
            "MAV_GCS_SYSID": self.mav.source_system,
            "RC12_OPTION": 46, # enable/disable rc overrides
        })
        self.reboot_sitl()

        self.change_mode("MANUAL")
        self.wait_ready_to_arm()
        self.arm_vehicle()
        self.progress("start moving forward a little")
        normal_rc_throttle = 1700
        self.set_rc(3, normal_rc_throttle)
        self.wait_groundspeed(5, 100)

        self.progress("allow overrides")
        self.set_rc(12, 2000)

        self.progress("now override to stop")
        throttle_override_normalized = 0
        expected_throttle = 0 # in VFR_HUD

        tstart = self.get_sim_time_cached()
        while True:
            if self.get_sim_time_cached() - tstart > 10:
                raise AutoTestTimeoutException("Did not reach speed")
            self.progress("Sending normalized throttle of %d" % (throttle_override_normalized,))
            self.mav.mav.manual_control_send(
                1, # target system
                32767, # x (pitch)
                32767, # y (roll)
                throttle_override_normalized, # z (thrust)
                32767, # r (yaw)
                0) # button mask

            m = self.assert_receive_message('VFR_HUD')
            want_speed = 2.0
            self.progress("Speed=%f want=<%f  throttle=%u want=%u" %
                          (m.groundspeed, want_speed, m.throttle, expected_throttle))
            if m.groundspeed < want_speed and m.throttle == expected_throttle:
                break

        self.progress("now override to stop - but set the switch on the RC transmitter to deny overrides; this should send the speed back up to 5 metres/second")  # noqa
        self.set_rc(12, 1000)

        throttle_override_normalized = 500
        expected_throttle = 36 # in VFR_HUD, corresponding to normal_rc_throttle adjusted for channel min/max

        tstart = self.get_sim_time_cached()
        while True:
            if self.get_sim_time_cached() - tstart > 10:
                raise AutoTestTimeoutException("Did not stop")
            self.progress("Sending normalized throttle of %u" % (throttle_override_normalized,))
            self.mav.mav.manual_control_send(
                1, # target system
                32767, # x (pitch)
                32767, # y (roll)
                throttle_override_normalized, # z (thrust)
                32767, # r (yaw)
                0) # button mask

            m = self.assert_receive_message('VFR_HUD')
            want_speed = 5.0

            self.progress("Speed=%f want=>%f  throttle=%u want=%u" %
                          (m.groundspeed, want_speed, m.throttle, expected_throttle))
            if m.groundspeed > want_speed and m.throttle == expected_throttle:
                break

        # re-enable RC overrides
        self.set_rc(12, 2000)

        # check we revert to normal RC inputs when gcs overrides cease:
        self.progress("Waiting for RC to revert to normal RC input")
        self.wait_rc_channel_value(3, normal_rc_throttle, timeout=10)

        self.disarm_vehicle()
