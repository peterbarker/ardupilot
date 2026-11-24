'''
Plane autotest: JumpBestLandSequence

AP_FLAKE8_CLEAN
'''

from pymavlink import mavutil

from vehicle_test_suite import NotAchievedException

from test_discovery import TestGroup
from test_discovery import testcase


class JumpBestLandSequence(TestGroup):
    @testcase(bucket="PlaneTests1b")
    def JumpBestLandSequence(self):
        '''Verify a failsafe lets the mission carry on to land when a jump makes that the shortest way'''

        # with the FAILSAFE_TO_BEST_LANDING mission option a failsafe compares
        # the distance to land via the nearest DO_LAND_START with the distance
        # to land by carrying on with the mission.  Here the jump skips the
        # waypoints that follow the DO_LAND_START, so carrying on is much
        # shorter.  If the distance calculation does not follow the jump it
        # counts those waypoints, decides the mission is the longer way and
        # sends the plane off to fly them, 3km north of home
        alt = 60
        self.set_parameters({
            "MIS_OPTIONS": 2,       # FAILSAFE_TO_BEST_LANDING
            "RTL_AUTOLAND": 1,      # required when the mission has a DO_LAND_START
            "BATT_LOW_VOLT": 11,
            "BATT_LOW_TIMER": 1,
            "BATT_FS_LOW_ACT": 2,   # land
        })
        self.upload_simple_relhome_mission([
            (mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, alt, {'p1': 15}),                    # 1
            (mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 600, alt),                             # 2
            self.create_MISSION_ITEM_INT(mavutil.mavlink.MAV_CMD_DO_JUMP_TAG, p1=1, p2=1),   # 3 jump to the tag, once
            (mavutil.mavlink.MAV_CMD_DO_LAND_START, 0, 300, 0),                              # 4
            (mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 3000, 600, alt),                          # 5 skipped by the jump
            (mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 3000, -2000, alt),                        # 6 skipped by the jump
            self.create_MISSION_ITEM_INT(mavutil.mavlink.MAV_CMD_JUMP_TAG, p1=1),            # 7 the tag
            (mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 250, alt),                             # 8
            (mavutil.mavlink.MAV_CMD_NAV_LAND, 0, 0, 0),                                     # 9
        ])

        self.change_mode("AUTO")
        self.wait_ready_to_arm()
        self.arm_vehicle()

        # fail the battery while the plane is airborne and heading for waypoint 2
        self.wait_current_waypoint(2, timeout=120)
        self.wait_altitude(30, 200, relative=True, timeout=120)
        self.set_parameter("SIM_BATT_VOLTAGE", 9.5)

        # the plane should carry on, jump over waypoints 5 and 6 and land at
        # home.  Position is checked rather than the reported waypoint, as the
        # failure is the plane flying to the wrong place
        home = self.home_position_as_location()
        tstart = self.get_sim_time()
        while self.armed():
            if self.get_sim_time() - tstart > 300:
                raise NotAchievedException("Plane did not land")
            dist = self.get_distance(home, self.get_location())
            if dist > 1500:
                raise NotAchievedException(
                    "Plane flew %.0fm from home after the failsafe, off to the waypoints the jump skips" % dist)
            self.delay_sim_time(1, "waiting for the plane to land")

        # the landing itself is not precise, but the waypoints the jump
        # skips are 3km away
        dist = self.get_distance(home, self.get_location())
        if dist > 1000:
            raise NotAchievedException("Plane landed %.0fm from home" % dist)
