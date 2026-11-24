'''
Copter autotest: clear_roi

AP_FLAKE8_CLEAN
'''

from pymavlink import mavutil

from vehicle_test_suite import AltFrame

from test_discovery import TestGroup
from test_discovery import testcase


class clear_roi(TestGroup):
    @testcase(bucket="CopterTests2d")
    def clear_roi(self):
        '''ensure three commands that clear ROI are equivalent'''

        # 8000m so the vehicle never reaches the waypoint: the thirteen heading
        # waits below are capped at 30s each and the twelve command ACKs at 10s,
        # 5100m at the default WP_SPD, from within 500m of home.  A stopped
        # vehicle holds its last yaw.
        self.upload_simple_relhome_mission([
            (mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,     0, 0, 20),
            (mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 8000, 0, 20), # directly North, i.e. 0 degrees
        ])

        self.set_parameter("AUTO_OPTIONS", 3)
        self.change_mode('AUTO')
        self.wait_ready_to_arm()
        self.arm_vehicle()
        home_loc = self.get_location()

        self.wait_distance_to_home(150, 500, timeout=120)
        self.wait_heading(0)

        cmd_ids = [
            mavutil.mavlink.MAV_CMD_DO_SET_ROI,
            mavutil.mavlink.MAV_CMD_DO_SET_ROI_LOCATION,
            mavutil.mavlink.MAV_CMD_DO_SET_ROI_NONE,
        ]
        for command in self.run_cmd, self.run_cmd_int:
            for cmd_id in cmd_ids:
                # Set an ROI at the Home location, expect to point at Home
                self.run_cmd(mavutil.mavlink.MAV_CMD_DO_SET_ROI_LOCATION,
                             p5=home_loc.lat,
                             p6=home_loc.lng,
                             p7=home_loc.get_alt_m(AltFrame.ABSOLUTE))
                self.wait_heading(180)

                # Clear the ROI, expect to point along the flight path again
                self.progress("Clear ROI using %s(%d)" % (command.__name__, cmd_id))
                command(cmd_id)
                self.wait_heading(0)

        self.land_and_disarm()

        # we are not at the home location - reboot so the next test starts there
        self.reboot_sitl()
