'''
Plane autotest: DO_REPOSITION

AP_FLAKE8_CLEAN
'''

from pymavlink import mavutil

from vehicle_test_suite import NotAchievedException

from test_discovery import TestGroup
from test_discovery import testcase


class DO_REPOSITION(TestGroup):
    @testcase(bucket="PlaneTests1b")
    def DO_REPOSITION(self):
        '''Test mavlink DO_REPOSITION command'''
        self.progress("Takeoff")
        self.takeoff(alt=50)
        self.set_rc(3, 1500)
        self.progress("Entering guided and flying somewhere constant")
        self.change_mode("GUIDED")
        loc = self.get_location()
        self.location_offset_ne(loc, 500, 500)

        new_alt = 100
        self.run_cmd_int(
            mavutil.mavlink.MAV_CMD_DO_REPOSITION,
            p5=int(loc.lat * 1e7),
            p6=int(loc.lng * 1e7),
            p7=new_alt,    # alt
            frame=mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
        )
        self.wait_altitude(new_alt-10, new_alt, timeout=30, relative=True)

        self.install_terrain_handlers_context()

        self.location_offset_ne(loc, 500, 500)
        terrain_height_wanted = 150
        self.run_cmd_int(
            mavutil.mavlink.MAV_CMD_DO_REPOSITION,
            0,
            0,
            0,
            0,
            int(loc.lat*1e7),
            int(loc.lng*1e7),
            terrain_height_wanted,    # alt
            frame=mavutil.mavlink.MAV_FRAME_GLOBAL_TERRAIN_ALT,
        )

        # move to specific terrain-relative altitude and hold for <n> seconds
        tstart = self.get_sim_time_cached()
        achieve_start = None
        tr = None
        while True:
            if self.get_sim_time_cached() - tstart > 120:
                raise NotAchievedException("Did not move to correct terrain alt")

            m = self.assert_receive_message('TERRAIN_REPORT')
            tr = m
            terrain_height_achieved = m.current_height
            self.progress("terrain_alt=%f want=%f" %
                          (terrain_height_achieved, terrain_height_wanted))
            if m is None:
                continue
            if abs(terrain_height_wanted - terrain_height_achieved) > 5:
                if achieve_start is not None:
                    self.progress("Achieve stop")
                    achieve_start = None
            elif achieve_start is None:
                self.progress("Achieve start")
                achieve_start = self.get_sim_time_cached()
            if achieve_start is not None:
                if self.get_sim_time_cached() - achieve_start > 10:
                    break
        m = self.assert_receive_message('GLOBAL_POSITION_INT')
        self.progress("TR: %s" % tr)
        self.progress("GPI: %s" % m)
        min_delta = 4
        delta = abs(m.relative_alt/1000.0 - tr.current_height)
        if abs(delta < min_delta):
            raise NotAchievedException("Expected altitude delta (want=%f got=%f)" %
                                       (min_delta, delta))

        self.fly_home_land_and_disarm(timeout=180)
