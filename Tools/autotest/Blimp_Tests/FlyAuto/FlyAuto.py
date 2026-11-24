'''
Blimp autotest: FlyAuto

AP_FLAKE8_CLEAN
'''

from pymavlink import mavutil

from test_discovery import TestGroup
from test_discovery import testcase


class FlyAuto(TestGroup):
    @testcase(bucket="Blimp")
    def FlyAuto(self):
        '''test AUTO mode with a small mission '''
        self.customise_SITL_commandline(
            [],
            model='blimp-motor',
            defaults_filepath=self.model_defaults_filepath('blimp-motor'),
        )

        alt = 1  # metres above home (corner/default)
        alt_high = 1.5
        alt_low = 0.5
        size = 5  # metres square

        self.progress("Uploading simple relhome mission.")

        # build a simple relative-to-home mission using helper
        # items are tuples: (MAV_CMD, metres_north, metres_east, alt, [opts])
        self.upload_simple_relhome_mission([
            (mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0,      0,      alt),
            (mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, size/2, -1,     alt_high),
            (mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, size,   0,      alt),
            (mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, size-1, size/2, alt_low),
            (mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, size,   size,   alt),
            (mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, size/2, size-1, alt_high),
            (mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0,      size,   alt),
            (mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0,      0,      alt),
        ])

        # ensure we start from first waypoint
        self.set_current_waypoint(0, check_afterwards=False)

        # switch to AUTO and arm
        self.change_mode('AUTO')
        self.wait_ready_to_arm()
        self.arm_vehicle()

        self.wait_statustext("Mission Complete", timeout=100)
        # always disarm at end of test
        self.disarm_vehicle()
