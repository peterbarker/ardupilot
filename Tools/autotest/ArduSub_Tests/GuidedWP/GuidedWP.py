'''
Sub autotest: GuidedWP

AP_FLAKE8_CLEAN
'''

from pymavlink import mavutil

from pysim import util
from vehicle_test_suite import NotAchievedException

from test_discovery import TestGroup
from test_discovery import testcase

from ardusub import RE_TR_SEARCH


class GuidedWP(TestGroup):
    @testcase(bucket="Sub")
    def GuidedWP(self):
        """Test Guided_WP mode"""

        pos_mode = (mavutil.mavlink.POSITION_TARGET_TYPEMASK_VX_IGNORE |
                    mavutil.mavlink.POSITION_TARGET_TYPEMASK_VY_IGNORE |
                    mavutil.mavlink.POSITION_TARGET_TYPEMASK_VZ_IGNORE |
                    mavutil.mavlink.POSITION_TARGET_TYPEMASK_AX_IGNORE |
                    mavutil.mavlink.POSITION_TARGET_TYPEMASK_AY_IGNORE |
                    mavutil.mavlink.POSITION_TARGET_TYPEMASK_AZ_IGNORE |
                    mavutil.mavlink.POSITION_TARGET_TYPEMASK_YAW_IGNORE |
                    mavutil.mavlink.POSITION_TARGET_TYPEMASK_YAW_RATE_IGNORE)

        seafloor_depth = 50
        speed = 0.5

        # Load the synthetic seafloor, this will push a context and reboot
        self.prepare_synthetic_seafloor_test(seafloor_depth, 10)  # rf_target is not used

        self.set_parameter('WP_SPD', speed)

        # the vehicle reports wp_nav's distance-to-destination in
        # NAV_CONTROLLER_OUTPUT; we use it to confirm our target has been
        # accepted.  Ask for it before diving; there is no depth hold
        # between the dive and entering GUIDED, so sim time spent here is
        # spent floating away from the dived-to depth
        self.context_set_message_rate_hz('NAV_CONTROLLER_OUTPUT', 10)

        self.dive(-15, mode='ALT_HOLD')

        # GLOBAL_POSITION_INT will be our clock
        self.context_set_message_rate_hz('GLOBAL_POSITION_INT', 10)

        # Run back and forth between 2 locations
        distance = 30
        timeout = distance / speed + 20  # Add time to accelerate and decelerate

        runs = [{
            # Hold depth as the terrain rises
            'frame': mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
            'bearing': 180,
            'target_alt': -15,  # Altitude above origin
            'max_error_allowed': 1.0,
        }, {
            # Hold range as the terrain falls
            'frame': mavutil.mavlink.MAV_FRAME_GLOBAL_TERRAIN_ALT,
            'bearing': 0,
            'target_alt': 25,  # Distance to seafloor
            'max_error_allowed': 1.0,
        }]

        # Stay in GUIDED mode for the duration
        self.change_mode('GUIDED')

        for run in runs:
            msg = self.assert_receive_message('GLOBAL_POSITION_INT')
            start_loc = (msg.lat * 1e-7, msg.lon * 1e-7)
            dest_loc = util.gps_newpos(start_loc[0], start_loc[1], run['bearing'], distance)

            # current_alt = range (distance to seafloor) or altitude (distance above origin), depending on the frame
            current_alt = None
            max_error = 0.0

            # Send the target until the reported distance-to-destination
            # shows it has become the current destination.  The vehicle
            # can silently reject the target (e.g. a single bad
            # rangefinder reading invalidates the terrain frame for an
            # instant) and there is no acknowledgement to check, so a
            # single send is not enough.
            accept_start = self.get_sim_time()
            accepted = False
            while not accepted:
                if self.get_sim_time_cached() - accept_start > 30:
                    raise NotAchievedException(f'Frame {run["frame"]} target not accepted')
                self.mav.mav.set_position_target_global_int_send(
                    0, 1, 1, run['frame'], pos_mode,
                    int(dest_loc[0] * 1e7), int(dest_loc[1] * 1e7), run['target_alt'],
                    0, 0, 0, 0, 0, 0, 0, 0)
                # wait a little for the new distance-to-destination to be
                # reported before concluding the target was rejected
                deadline = self.get_sim_time_cached() + 1
                while self.get_sim_time_cached() < deadline:
                    msg = self.assert_receive_message('NAV_CONTROLLER_OUTPUT')
                    if abs(msg.wp_dist - distance) < 5:
                        accepted = True
                        break

            start_time = self.get_sim_time()
            while True:
                msg = self.assert_receive_message(['GLOBAL_POSITION_INT', 'STATUSTEXT'])
                # Get ground truth (sans noise) from the terrain generator
                if msg.get_type() == 'STATUSTEXT':
                    if run['frame'] == mavutil.mavlink.MAV_FRAME_GLOBAL_TERRAIN_ALT:
                        match = RE_TR_SEARCH.search(msg.text)
                        if match:
                            current_alt = float(match.group(1))
                    continue

                current_loc = (msg.lat * 1e-7, msg.lon * 1e-7)
                distance_remaining = util.gps_distance(
                    dest_loc[0], dest_loc[1],
                    current_loc[0], current_loc[1])

                if run['frame'] == mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT:
                    current_alt = msg.relative_alt * 0.001

                if distance_remaining < 0.5:
                    self.progress(f'Frame {run["frame"]} reached destination at time '
                                  f'{self.get_sim_time_cached() - start_time}, max_error {max_error}')
                    break
                elif self.get_sim_time_cached() - start_time > timeout:
                    raise NotAchievedException(f'Frame {run["frame"]} took too long to reach the destination')

                if current_alt is None:
                    continue

                alt_error = abs(current_alt - run['target_alt'])
                if alt_error > run['max_error_allowed']:
                    raise NotAchievedException(
                        f'Alt incorrect on frame {run["frame"]}: want {run["target_alt"]:.2f} '
                        f'(+/- {run["max_error_allowed"]:.2f}) got={current_alt:.2f}')

                if alt_error > max_error:
                    max_error = alt_error
        self.disarm_vehicle()
