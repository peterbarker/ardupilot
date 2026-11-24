'''
Copter autotest: AirModeLanding

AP_FLAKE8_CLEAN
'''

from pymavlink import mavutil

from vehicle_test_suite import NotAchievedException

from test_discovery import TestGroup
from test_discovery import testcase


class AirModeLanding(TestGroup):
    @testcase(bucket="CopterTests2d")
    def AirModeLanding(self):
        '''test air-mode holds off landing detection rather than landing in mid-air'''

        def land_detector_flight():
            """the land detector must not decide the vehicle has landed just
            because the throttle stick is at zero in air-mode"""

            self.change_mode('GUIDED')
            self.user_takeoff(alt_min=100, timeout=180)

            # a strong updraft, so that with the throttle stick at zero the
            # vehicle sits in equilibrium instead of falling.  That makes
            # everything the land detector looks at - motors at their lower
            # limit, no acceleration, no descent - say "landed", leaving the
            # air-mode land-detector timeout as the only thing standing
            # between us and a landing detected in mid-air:
            self.set_parameters({
                "SIM_WIND_T": 1,        # no gradient; we are a long way up
                "SIM_WIND_DIR_Z": 90,   # straight up
                "SIM_WIND_SPD": 16,     # a little over terminal velocity
            })
            # reach that equilibrium before taking manual control, so that the
            # land detector's accelerating/descending checks are already
            # satisfied and only its timeout is left to hold it off:
            self.wait_climbrate(-1, 1, minimum_duration=2, timeout=30)

            # the throttle stick has been at zero throughout, so the land
            # detector can start counting the moment we take manual control:
            self.zero_throttle()
            self.change_mode('STABILIZE')

            # air-mode delays landing detection, it does not prevent it.  If
            # we never detect a landing then this scenario did not look like a
            # landing at all and we have tested nothing:
            self.context_set_message_rate_hz(
                mavutil.mavlink.MAVLINK_MSG_ID_EXTENDED_SYS_STATE, 10)
            self.wait_extended_sys_state(
                mavutil.mavlink.MAV_VTOL_STATE_MC,
                mavutil.mavlink.MAV_LANDED_STATE_ON_GROUND,
                timeout=10)
            self.disarm_vehicle(force=True)

            # find how long the land detector took to call this a landing.
            # Use the onboard log; MAVLink timing is too coarse to tell
            # LAND_DETECTOR_TRIGGER_SEC (1s) from
            # LAND_AIRMODE_DETECTOR_TRIGGER_SEC (3s) apart:
            stabilize_us = None
            land_complete_us = None
            dfreader = self.dfreader_for_current_onboard_log()
            while True:
                m = dfreader.recv_match(type=['MODE', 'EV'])
                if m is None:
                    break
                if m.get_type() == 'MODE':
                    if m.ModeNum == self.get_mode_from_mode_mapping('STABILIZE'):
                        # start again from this mode change:
                        stabilize_us = m.TimeUS
                        land_complete_us = None
                    continue
                if (m.Id == 18 and  # LogEvent::LAND_COMPLETE
                        stabilize_us is not None and
                        land_complete_us is None):
                    land_complete_us = m.TimeUS

            if stabilize_us is None:
                raise NotAchievedException("Did not find STABILIZE in the log")
            if land_complete_us is None:
                raise NotAchievedException("Did not find LAND_COMPLETE in the log")
            delay = (land_complete_us - stabilize_us) * 1.0e-6
            self.progress("landing detected %.2fs after taking manual control" % delay)
            if delay < 2:
                raise NotAchievedException(
                    "Landing detected in mid-air %.2fs after taking manual "
                    "control at zero throttle (air-mode should hold this off "
                    "for %.0fs)" % (delay, 3))

            self.zero_throttle()
            self.reboot_sitl()

        def arm_with_aux_switch():
            """arm normally, then turn air-mode on with the AIRMODE aux switch"""
            self.set_rc_from_map({7: 1000, 8: 1000})
            self.wait_ready_to_arm()
            self.zero_throttle()
            self.arm_vehicle()
            self.set_rc(8, 2000)

        def arm_with_arming_switch():
            """arm with the ARMDISARM_AIRMODE switch, which implies air-mode"""
            self.set_rc_from_map({7: 1000, 8: 1000})
            self.wait_ready_to_arm()
            self.zero_throttle()
            self.arm_motors_with_switch(7)

        self.set_parameters({
            "RC7_OPTION": 154,  # ARMDISARM_AIRMODE
            "RC8_OPTION": 84,   # AIRMODE
        })

        for (description, arm_method) in [
                ("the AIRMODE aux switch", arm_with_aux_switch),
                ("arming with the ARMDISARM_AIRMODE switch", arm_with_arming_switch),
        ]:
            self.start_subtest("no mid-air landing detection with %s" % description)
            arm_method()
            land_detector_flight()
