'''
Copter autotest: mission_NAV_LOITER_TURNS_direction

AP_FLAKE8_CLEAN
'''

from pymavlink import mavutil

from vehicle_test_suite import AutoTestTimeoutException
from vehicle_test_suite import Location
from vehicle_test_suite import NotAchievedException

from test_discovery import TestGroup
from test_discovery import testcase


class mission_NAV_LOITER_TURNS_direction(TestGroup):
    @testcase(bucket="CopterTests2c")
    def mission_NAV_LOITER_TURNS_direction(self):
        '''LOITER_TURNS orbits the way the sign of param3 asks'''
        radius = 20
        self.set_parameters({
            'AUTO_OPTIONS': 3,
            'WP_SPD': 5,
        })
        self.wait_ready_to_arm()
        circle_centre_loc = self.offset_location_ne(self.get_location(), 2 * radius, 0)
        self.upload_simple_relhome_mission([
            (mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 20),
            self.create_MISSION_ITEM_INT(
                mavutil.mavlink.MAV_CMD_NAV_LOITER_TURNS,
                p1=1,
                p3=-radius,  # a negative radius asks for a counter-clockwise orbit
                x=int(circle_centre_loc.lat*1e7),
                y=int(circle_centre_loc.lng*1e7),
                z=20,
                frame=mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
            ),
            (mavutil.mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH, 0, 0, 0),
        ])
        self.change_mode('AUTO')
        self.wait_ready_to_arm()
        self.arm_vehicle()

        # only start measuring once the orbit itself has been announced, so the leg out to the
        # circle edge does not contribute
        self.wait_statustext("Mission: circling", timeout=120)
        tstart = self.get_sim_time()

        # accumulate the bearing from the circle centre out to the vehicle.  Going clockwise makes
        # that bearing increase and counter-clockwise makes it decrease, so the sign of the total
        # is the direction flown
        total_angle_deg = 0.0
        last_bearing_deg = None
        while True:
            if self.get_sim_time_cached() - tstart > 120:
                raise AutoTestTimeoutException("orbit did not complete")
            m = self.assert_receive_message(
                ['GLOBAL_POSITION_INT', 'MISSION_CURRENT'],
                timeout=5,
            )
            if m.get_type() == 'GLOBAL_POSITION_INT':
                pos = Location.latlon_only(m.lat * 1e-7, m.lon * 1e-7)
                bearing_deg = self.get_bearing(circle_centre_loc, pos)
                if last_bearing_deg is not None:
                    delta_deg = bearing_deg - last_bearing_deg
                    if delta_deg > 180:
                        delta_deg -= 360
                    elif delta_deg < -180:
                        delta_deg += 360
                    total_angle_deg += delta_deg
                last_bearing_deg = bearing_deg
            elif m.seq >= 3:
                break

        if abs(total_angle_deg + 360) > 80:
            raise NotAchievedException(
                "Orbit swept %.0fdeg, expected -360deg (negative param3 = counter-clockwise)" %
                total_angle_deg)

        self.wait_disarmed()
