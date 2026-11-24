'''
Plane autotest: TerrainRally

AP_FLAKE8_CLEAN
'''

from vehicle_test_suite import AltFrame
from vehicle_test_suite import Location
from vehicle_test_suite import NotAchievedException

from test_discovery import TestGroup
from test_discovery import testcase


class TerrainRally(TestGroup):
    @testcase(bucket="PlaneTests1b")
    def TerrainRally(self):
        """ Tests terrain follow with a rally point """
        self.context_push()
        self.install_terrain_handlers_context()

        def terrain_following_above_80m(mav, m):
            if m.get_type() == 'TERRAIN_REPORT':
                if m.current_height < 50:
                    raise NotAchievedException(
                        "TERRAIN_REPORT.current_height below 50m %fm" % m.current_height)
            if m.get_type() == 'VFR_HUD':
                if m.groundspeed < 2:
                    raise NotAchievedException("hit ground")

        def terrain_wait_path(loc1, loc2, steps):
            '''wait till we have terrain for N steps from loc1 to loc2'''
            tstart = self.get_sim_time_cached()
            self.progress("Waiting for terrain data")
            while True:
                now = self.get_sim_time_cached()
                if now - tstart > 60:
                    raise NotAchievedException("Did not get correct required terrain")
                for i in range(steps+1):
                    lat = loc1.lat + i * (loc2.lat-loc1.lat)/steps
                    lon = loc1.lng + i * (loc2.lng-loc1.lng)/steps
                    self.mav.mav.terrain_check_send(int(lat*1.0e7), int(lon*1.0e7))

                report = self.assert_receive_message('TERRAIN_REPORT', timeout=60)
                self.progress("Terrain pending=%u" % report.pending)
                if report.pending == 0:
                    break
                self.delay_sim_time(1, reason="vehicle to fetch terrain data")
            self.progress("Got required terrain")

        self.wait_ready_to_arm()
        homeloc = self.get_location()

        guided_loc = Location(-35.39723762, 149.07284612, 99.0, AltFrame.ABOVE_TERRAIN)
        # uploaded relative to home; TERRAIN_FOLLOW makes RTL follow
        # terrain on the way, but it arrives at the same AMSL height
        rally_loc_relhome = Location(-35.3654952000, 149.1558698000, 100, AltFrame.ABOVE_HOME)
        rally_loc = rally_loc_relhome.copy()
        rally_loc.set_alt_m(100, AltFrame.ABOVE_TERRAIN)

        terrain_wait_path(homeloc, rally_loc, 10)
        terrain_wait_path(homeloc, guided_loc, 10)

        # set a rally point to the west of home
        self.upload_rally_points_from_locations([rally_loc_relhome])

        self.set_parameter("TKOFF_ALT", 100)
        self.change_mode("TAKEOFF")
        self.wait_ready_to_arm()
        self.arm_vehicle()
        self.set_parameter("TERRAIN_FOLLOW", 1)
        self.wait_altitude(90, 120, timeout=30, relative=True)
        self.progress("Done takeoff")

        self.install_message_hook_context(terrain_following_above_80m)

        self.change_mode("GUIDED")
        self.send_do_reposition(guided_loc)
        self.progress("Flying to guided location")
        self.wait_location(
            guided_loc,
            accuracy=200,
            timeout=600,
            height_accuracy=10,
        )

        self.progress("Reached guided location")
        self.set_parameter("RALLY_LIMIT_KM", 50)
        self.change_mode("RTL")
        self.progress("Flying to rally point")
        self.wait_location(
            rally_loc_relhome,
            accuracy=200,
            timeout=600,
            height_accuracy=10,
            minimum_duration=10,
        )
        self.progress("Reached rally point with TERRAIN_FOLLOW")

        # Fly back to guided location
        self.change_mode("GUIDED")
        self.send_do_reposition(guided_loc)
        self.progress("Flying to back to guided location")

        # Disable terrain following and re-load rally point with relative to terrain altitude
        self.set_parameter("TERRAIN_FOLLOW", 0)

        self.upload_rally_points_from_locations([rally_loc])

        # Once back at guided location re-trigger RTL
        self.wait_location(
            guided_loc,
            accuracy=200,
            timeout=600,
            height_accuracy=10,
        )

        self.change_mode("RTL")
        self.progress("Flying to rally point")
        self.wait_location(
            rally_loc,
            accuracy=200,
            timeout=600,
            height_accuracy=10,
            minimum_duration=10,
        )
        self.progress("Reached rally point with terrain alt frame")

        self.context_pop()
        self.disarm_vehicle(force=True)
        self.reboot_sitl()
