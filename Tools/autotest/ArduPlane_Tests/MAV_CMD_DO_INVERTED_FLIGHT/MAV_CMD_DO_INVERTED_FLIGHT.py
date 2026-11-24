'''
Plane autotest: MAV_CMD_DO_INVERTED_FLIGHT

AP_FLAKE8_CLEAN
'''

from pymavlink import mavutil

from vehicle_test_suite import NotAchievedException

from test_discovery import TestGroup
from test_discovery import testcase


class MAV_CMD_DO_INVERTED_FLIGHT(TestGroup):
    @testcase(bucket="PlaneTests1a")
    def MAV_CMD_DO_INVERTED_FLIGHT(self):
        '''fly upside-down mission item'''
        alt = 30
        wps = self.create_simple_relhome_mission([
            (mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, alt),
            (mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 400, 0, alt),
            self.create_MISSION_ITEM_INT(
                mavutil.mavlink.MAV_CMD_DO_INVERTED_FLIGHT,
                p1=1,
            ),
            (mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 800, 0, alt),
            self.create_MISSION_ITEM_INT(
                mavutil.mavlink.MAV_CMD_DO_INVERTED_FLIGHT,
                p1=0,
            ),
            (mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 1200, 0, alt),
            (mavutil.mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH, 0, 0, 0),
        ])
        self.check_mission_upload_download(wps)

        self.change_mode('AUTO')
        self.wait_ready_to_arm()

        self.arm_vehicle()

        self.wait_current_waypoint(2)  # upright flight
        self.wait_message_field_values("NAV_CONTROLLER_OUTPUT", {
            "nav_roll": 0,
            "nav_pitch": 0,
        }, epsilon=10)

        def check_altitude(mav, m):
            m_type = m.get_type()
            if m_type != 'GLOBAL_POSITION_INT':
                return
            if abs(30 - m.relative_alt * 0.001) > 15:
                raise NotAchievedException("Bad altitude while flying inverted")

        self.context_push()
        self.install_message_hook_context(check_altitude)

        self.wait_current_waypoint(4)  # inverted flight
        self.wait_message_field_values("NAV_CONTROLLER_OUTPUT", {
            "nav_roll": 180,
            "nav_pitch": 9,
        }, epsilon=10,)

        self.wait_current_waypoint(6)  # upright flight
        self.wait_message_field_values("NAV_CONTROLLER_OUTPUT", {
            "nav_roll": 0,
            "nav_pitch": 0,
        }, epsilon=10)

        self.context_pop()  # remove the check_altitude call

        self.wait_current_waypoint(7)

        self.fly_home_land_and_disarm()
