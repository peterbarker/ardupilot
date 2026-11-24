'''
Plane autotest: DO_REPOSITION_loiter_radius_and_direction

AP_FLAKE8_CLEAN
'''

from pymavlink import mavutil

from test_discovery import TestGroup
from test_discovery import testcase


class DO_REPOSITION_loiter_radius_and_direction(TestGroup):
    @testcase(bucket="PlaneTests1b")
    def DO_REPOSITION_loiter_radius_and_direction(self):
        '''test the loiter radius and direction parameters of MAV_CMD_DO_REPOSITION'''
        self.takeoff(50, mode='TAKEOFF')
        centre = self.offset_location_ne(self.home_position_as_location(), 600, 0)

        def get_yaw_rate():
            return self.assert_receive_message('ATTITUDE').yawspeed

        for direction, radius, min_yaw_rate, max_yaw_rate in (1, 150, -1, -0.03), (0, 250, 0.03, 1):
            self.start_subtest(f"direction={direction} radius={radius}")
            self.change_mode('LOITER')
            self.run_cmd_int(
                mavutil.mavlink.MAV_CMD_DO_REPOSITION,
                p2=mavutil.mavlink.MAV_DO_REPOSITION_FLAGS_CHANGE_MODE,
                p3=radius,
                p4=direction,  # 0 is clockwise, 1 counter-clockwise
                p5=int(centre.lat * 1e7),
                p6=int(centre.lng * 1e7),
                p7=50,
                frame=mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
            )
            self.wait_mode('GUIDED')
            self.wait_distance_to_location(centre, radius-30, radius+30, minimum_duration=20, timeout=120)
            self.wait_and_maintain_range(
                value_name="YawRate",
                minimum=min_yaw_rate,
                maximum=max_yaw_rate,
                current_value_getter=get_yaw_rate,
                minimum_duration=10,
                timeout=30,
            )

        self.fly_home_land_and_disarm()
