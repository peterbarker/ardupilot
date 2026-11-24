'''
Plane autotest: PID_TUNING_axes

AP_FLAKE8_CLEAN
'''

from pymavlink import mavutil

from vehicle_test_suite import NotAchievedException

from test_discovery import TestGroup
from test_discovery import testcase


class PID_TUNING_axes(TestGroup):
    @testcase(bucket="PlaneTests1b")
    def PID_TUNING_axes(self):
        '''check PID_TUNING is sent for the axes selected by GCS_PID_MASK'''
        self.set_parameter("GCS_PID_MASK", 1 | 2 | 4 | 8)  # roll, pitch, yaw, steer
        self.change_mode('FBWA')
        want_axes = set([
            mavutil.mavlink.PID_TUNING_ROLL,
            mavutil.mavlink.PID_TUNING_PITCH,
            mavutil.mavlink.PID_TUNING_YAW,
            mavutil.mavlink.PID_TUNING_STEER,
        ])
        axes = self.received_pid_tuning_axes()
        if axes != want_axes:
            raise NotAchievedException(f"Want axes {sorted(want_axes)} got {sorted(axes)}")

        self.start_subtest("not sent in MANUAL")
        self.change_mode('MANUAL')
        axes = self.received_pid_tuning_axes()
        if len(axes) != 0:
            raise NotAchievedException(f"Received PID_TUNING in MANUAL: {sorted(axes)}")

        self.start_subtest("landing PID sent only while landing")
        self.set_parameters({
            "GCS_PID_MASK": 16,  # landing
            "LAND_TYPE": 1,  # deepstall, which has a landing PID
            "LAND_DS_ELEV_PWM": 1661,
        })
        self.change_mode('FBWA')
        axes = self.received_pid_tuning_axes()
        if len(axes) != 0:
            raise NotAchievedException(f"Received PID_TUNING when not landing: {sorted(axes)}")
        land_loc = self.offset_location_ne(self.home_position_as_location(), -30, -210)
        # a mission with DO_LAND_START fails prearms with RTL_AUTOLAND disabled:
        self.set_parameter("RTL_AUTOLAND", 1)
        self.start_flying_simple_relhome_mission([
            (mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 100),
            (mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 110, -65, 100),
            self.create_MISSION_ITEM_INT(mavutil.mavlink.MAV_CMD_DO_LAND_START),
            self.create_MISSION_ITEM_INT(
                mavutil.mavlink.MAV_CMD_NAV_LAND,
                p4=1,
                x=int(land_loc.lat * 1e7),
                y=int(land_loc.lng * 1e7),
                z=60,
                frame=mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
            ),
        ])
        self.assert_receive_message(
            'PID_TUNING',
            condition=f'PID_TUNING.axis=={mavutil.mavlink.PID_TUNING_LANDING}',
            timeout=240,
        )
        self.disarm_wait(timeout=120)
