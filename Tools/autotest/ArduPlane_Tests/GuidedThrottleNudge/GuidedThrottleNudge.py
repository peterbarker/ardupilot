'''
Plane autotest: GuidedThrottleNudge

AP_FLAKE8_CLEAN
'''

from pymavlink import mavutil

from vehicle_test_suite import AltFrame

from test_discovery import TestGroup
from test_discovery import testcase


class GuidedThrottleNudge(TestGroup):
    @testcase(bucket="PlaneTests1c")
    def GuidedThrottleNudge(self):
        '''ensure THROTTLE_NUDGE nudges target airspeed in GUIDED mode'''
        # regression test for throttle-nudge being ignored in GUIDED (issue #26094)
        self.set_parameters({
            "THROTTLE_NUDGE": 1,
            "AIRSPEED_CRUISE": 22,
            "AIRSPEED_MAX": 32,
            "ARSPD_USE": 1,
            # keep the vehicle flying near-straight for a stable airspeed
            # reading while staying bounded near home
            "WP_LOITER_RAD": 250,
        })
        self.takeoff(alt=100, mode="TAKEOFF", timeout=120)
        self.set_rc(3, 1500)
        self.change_mode("GUIDED")
        # hold position near the current location so the vehicle loiters
        # (auto-throttle active) rather than flying away from home
        self.send_do_reposition(self.get_location(frame=AltFrame.ABOVE_HOME))
        self.delay_sim_time(10, reason="vehicle to establish GUIDED position")

        self.start_subtest("Neutral throttle: target airspeed sits at cruise")
        self.wait_airspeed(20, 24, minimum_duration=5, timeout=60)

        self.start_subtest("Full throttle: nudge raises target airspeed toward AIRSPEED_MAX")
        self.set_rc(3, 2000)
        self.wait_airspeed(28, 40, minimum_duration=5, timeout=60)

        self.start_subtest("Neutral throttle: nudge released, back to cruise")
        self.set_rc(3, 1500)
        self.wait_airspeed(20, 24, minimum_duration=5, timeout=60)

        self.start_subtest("Offboard guided speed active: throttle nudge stays suppressed")
        # an active offboard airspeed slew must keep suppressing the nudge
        self.run_cmd(
            mavutil.mavlink.MAV_CMD_GUIDED_CHANGE_SPEED,
            p1=0,   # SPEED_TYPE_AIRSPEED
            p2=26,  # target airspeed m/s (between cruise and AIRSPEED_MAX)
            p3=5,   # acceleration m/s/s
        )
        self.delay_sim_time(8, reason="offboard airspeed slew to take effect")
        self.set_rc(3, 2000)
        # with a slew active the nudge is suppressed, so the target holds at
        # the commanded 26 m/s and does not climb toward AIRSPEED_MAX (32)
        self.wait_airspeed(24, 28, minimum_duration=8, timeout=40)
        self.set_rc(3, 1500)

        self.reboot_sitl(force=True)
