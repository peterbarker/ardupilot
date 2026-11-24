'''
Plane autotest: TerrainMissionInterrupt

AP_FLAKE8_CLEAN
'''

from pymavlink import mavutil

from vehicle_test_suite import AltFrame
from vehicle_test_suite import NotAchievedException

from test_discovery import TestGroup
from test_discovery import testcase


class TerrainMissionInterrupt(TestGroup):
    @testcase(bucket="PlaneTests1b")
    def TerrainMissionInterrupt(self):
        '''Test terrain following when resuming a mission'''
        self.install_terrain_handlers_context()

        self.load_mission("ap-terrain.txt")

        self.wait_ready_to_arm()
        self.arm_vehicle()

        self.set_parameter('TECS_RLL2THR', 2)

        # Keep track of the maximum terrain alt.
        global max_terrain_alt
        max_terrain_alt = 0

        def record_maxalt(mav, m):
            global max_terrain_alt
            if m.get_type() != 'TERRAIN_REPORT':
                return
            if m.current_height > max_terrain_alt:
                max_terrain_alt = m.current_height

        self.context_push()

        self.set_parameter("WP_RADIUS", 100)  # Ensure the aircraft will get within 100.0m of the waypoint.

        # Start the mission.
        self.set_current_waypoint(0, check_afterwards=False)
        self.change_mode('AUTO')

        # After waypoint 2, go to GUIDED.
        self.wait_waypoint(3, 3, max_dist_to_final_wp_m=3150, timeout=600)
        self.progress("Entering guided and flying somewhere constant")
        self.change_mode("GUIDED")
        new_alt = 280
        loc = self.get_location()
        self.location_offset_ne(loc, 350, 0)
        loc.set_alt_m(new_alt, AltFrame.ABOVE_HOME)
        self.run_cmd_int(
            mavutil.mavlink.MAV_CMD_DO_REPOSITION,
            p5=int(loc.lat * 1e7),
            p6=int(loc.lng * 1e7),
            p7=new_alt,  # alt
            frame=mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
        )

        # Resume auto when we are close to the GUIDED waypoint and start tracking maximum terrain alt.
        self.wait_location(loc, accuracy=100, height_accuracy=10)  # based on loiter radius
        self.change_mode('AUTO')
        self.install_message_hook_context(record_maxalt)

        self.wait_waypoint(3, 3, max_dist_to_final_wp_m=100, timeout=600)

        self.context_pop()

        # We've flown enough.
        self.disarm_vehicle(force=True)
        self.reboot_sitl()

        # self.fly_mission_waypoints(num_wp-1, mission_timeout=600)
        if max_terrain_alt > 120:
            raise NotAchievedException("Did not follow terrain")
