'''
Plane autotest: TakeoffGround

AP_FLAKE8_CLEAN
'''

import math

from vehicle_test_suite import NotAchievedException

from test_discovery import TestGroup
from test_discovery import testcase


class TakeoffGround(TestGroup):
    def takeoff_ground(self, gnd_pitch):
        '''Do a rolling TAKEOFF with the given TKOFF_GND_PITCH.'''

        self.set_parameters({
            "TKOFF_ROTATE_SPD": 15.0,
            "TKOFF_GND_PITCH": gnd_pitch,
            "TKOFF_PLIM_SEC": 0,  # Ensure TKOFF_PLIM_SEC doesn't interfere with the test.
            "TKOFF_DIST": 500,  # Ensure stall prevention doesn't interfere with the test.
            "TKOFF_ALT": 100  # Ditto
        })
        self.change_mode("TAKEOFF")

        self.wait_ready_to_arm()
        self.arm_vehicle()

        # Check that we demand minimum pitch below rotation speed.
        self.wait_groundspeed(8, 10)
        m = self.assert_receive_message('NAV_CONTROLLER_OUTPUT', timeout=5)
        nav_pitch = m.nav_pitch
        if abs(nav_pitch - gnd_pitch) > 0.1:
            raise NotAchievedException(f"Did not achieve correct takeoff pitch ({nav_pitch}).")
        ground_roll_time = self.get_sim_time()

        # Check whether we've achieved correct target pitch after rotation.
        self.wait_groundspeed(24, 25)
        m = self.assert_receive_message('NAV_CONTROLLER_OUTPUT', timeout=5)
        nav_pitch = m.nav_pitch
        if nav_pitch > 15.1 or nav_pitch < 14.9:
            raise NotAchievedException(f"Did not achieve correct takeoff pitch ({nav_pitch}).")

        self.fly_home_land_and_disarm()

        # Check that the TECS pitch limits were the ground pitch in the
        # last second before the ground roll check.
        dfreader = self.dfreader_for_current_onboard_log()
        count = 0
        while True:
            m = dfreader.recv_match(type='TECS')
            if m is None:
                break
            t = m.TimeUS * 1e-6
            if t < ground_roll_time - 1 or t > ground_roll_time:
                continue
            count += 1
            for (name, value) in (("pmin", m.pmin), ("pmax", m.pmax)):
                if abs(math.degrees(value) - gnd_pitch) > 0.1:
                    raise NotAchievedException(
                        f"TECS {name} is {math.degrees(value):.2f} deg, want {gnd_pitch}")
        if count == 0:
            raise NotAchievedException("No TECS messages in the ground roll")

    @testcase(bucket="PlaneTests1b")
    def TakeoffGround(self):
        '''Test a rolling TAKEOFF.'''
        self.takeoff_ground(5)

    @testcase(bucket="PlaneTests1b")
    def TakeoffGroundNegativePitch(self):
        '''Test a rolling TAKEOFF with a negative ground pitch.'''
        self.takeoff_ground(-2)
