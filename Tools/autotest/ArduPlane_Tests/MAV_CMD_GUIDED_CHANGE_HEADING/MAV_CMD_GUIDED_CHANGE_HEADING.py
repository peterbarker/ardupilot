'''
Plane autotest: MAV_CMD_GUIDED_CHANGE_HEADING

AP_FLAKE8_CLEAN
'''

from pymavlink import mavutil

from vehicle_test_suite import NotAchievedException

from test_discovery import TestGroup
from test_discovery import testcase


class MAV_CMD_GUIDED_CHANGE_HEADING(TestGroup):
    @testcase(bucket="PlaneTests1b")
    def MAV_CMD_GUIDED_CHANGE_HEADING(self):
        '''test flying headings with MAV_CMD_GUIDED_CHANGE_HEADING'''
        self.takeoff(50, mode='TAKEOFF')
        self.change_mode('GUIDED')
        # centripetal acceleration limit, giving about 45 degrees of bank:
        accel = 10

        self.start_subtest("vehicle heading")
        for heading in 90, 270:
            self.run_cmd_int(
                mavutil.mavlink.MAV_CMD_GUIDED_CHANGE_HEADING,
                p1=mavutil.mavlink.HEADING_TYPE_HEADING,
                p2=heading,
                p3=accel,
            )
            self.wait_heading(heading, accuracy=5, minimum_duration=10, timeout=60)

        self.start_subtest("course over ground")
        # a crosswind makes the course differ from the heading:
        self.set_parameters({
            "SIM_WIND_SPD": 8,
            "SIM_WIND_DIR": 0,
        })
        course = 90
        self.run_cmd_int(
            mavutil.mavlink.MAV_CMD_GUIDED_CHANGE_HEADING,
            p1=mavutil.mavlink.HEADING_TYPE_COURSE_OVER_GROUND,
            p2=course,
            p3=accel,
        )
        self.wait_and_maintain(
            value_name="CourseOverGround",
            target=course,
            current_value_getter=lambda: self.assert_receive_message('GPS_RAW_INT').cog * 0.01,
            validator=lambda value, target: self.heading_delta(value, target) <= 5,
            minimum_duration=10,
            timeout=60,
        )
        heading = self.get_heading()
        if self.heading_delta(heading, course) < 10:
            raise NotAchievedException(f"Expected heading ({heading}) to be crabbed away from course ({course})")
        self.set_parameter("SIM_WIND_SPD", 0)

        self.start_subtest("HEADING_TYPE_DEFAULT returns to normal GUIDED navigation")
        self.run_cmd_int(
            mavutil.mavlink.MAV_CMD_GUIDED_CHANGE_HEADING,
            p1=mavutil.mavlink.HEADING_TYPE_DEFAULT,
        )
        # the GUIDED loiter point is where GUIDED was entered, behind us:
        self.wait_heading(270, accuracy=20, timeout=60)

        self.fly_home_land_and_disarm()
