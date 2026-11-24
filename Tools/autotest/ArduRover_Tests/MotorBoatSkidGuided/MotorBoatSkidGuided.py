'''
Rover autotest: MotorBoatSkidGuided

AP_FLAKE8_CLEAN
'''

import math

from pymavlink import mavextra

from vehicle_test_suite import NotAchievedException

from test_discovery import TestGroup
from test_discovery import testcase


class MotorBoatSkidGuided(TestGroup):
    @testcase(bucket="Rover")
    def MotorBoatSkidGuided(self):
        '''ensure a skid-steering motorboat does not spin in GUIDED when retargeting directly behind'''
        model = "motorboat-skid"
        self.customise_SITL_commandline([],
                                        model=model,
                                        defaults_filepath=self.model_defaults_filepath(model))

        self.change_mode('LOITER')
        self.wait_ready_to_arm()

        # capture initial heading so we can place the two waypoints
        # along that axis - that way each trip is a true 180-deg turn
        att = self.assert_receive_message('ATTITUDE')
        initial_yaw_deg = math.degrees(att.yaw)
        here = self.get_location()
        north = math.cos(math.radians(initial_yaw_deg))
        east = math.sin(math.radians(initial_yaw_deg))
        loc_a = self.offset_location_ne(here, 25*north, 25*east)
        loc_b = self.offset_location_ne(here, -25*north, -25*east)

        self.change_mode('GUIDED')
        self.arm_vehicle()

        # accumulate per-sample yaw deltas (wrapped to +/-180) to
        # detect a spinning boat. With the SITL fix the skid-steer
        # max yaw rate is bounded to 5*pi ~= 16 deg/s; without it,
        # rates can exceed 300 deg/s when the bug fires.
        state = {'last_yaw_deg': None, 'total_change': 0.0}

        def yaw_hook(mav, m):
            if m.get_type() != 'ATTITUDE':
                return
            yaw = math.degrees(m.yaw)
            if state['last_yaw_deg'] is not None:
                delta = mavextra.wrap_180(yaw - state['last_yaw_deg'])
                state['total_change'] += abs(delta)
            state['last_yaw_deg'] = yaw

        self.install_message_hook_context(yaw_hook)

        # drive between two waypoints 50m apart along the initial
        # heading axis, pausing 10s at each. Each transition is a
        # 180-deg target relative to the boat's current heading,
        # which triggers the skid-steer spin bug in SITL when speed
        # has drifted slightly off zero.
        spin_limit_deg = 3 * 360
        for trip in range(4):
            target = loc_a if trip % 2 == 0 else loc_b
            name = 'A' if trip % 2 == 0 else 'B'
            self.progress("Trip %u: driving 50m to %s (180-deg turn)" % (trip, name))
            self.send_do_reposition(int(target.lat * 1e7), int(target.lng * 1e7))
            self.wait_location(target, accuracy=5, height_accuracy=None, timeout=60)
            self.progress("Dwelling at %s for 10s" % name)
            self.delay_sim_time(10, reason="heading stabilisation at waypoint")

        total = state['total_change']
        self.progress("Total yaw change %.0f deg over %u trips" % (total, trip + 1))
        if total > spin_limit_deg:
            raise NotAchievedException(
                "Boat spun: total yaw change %.0f deg exceeds %u deg" % (total, spin_limit_deg))

        self.disarm_vehicle(force=True)
