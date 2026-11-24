'''
Copter autotest: FenceMargin

AP_FLAKE8_CLEAN
'''

from pymavlink import mavutil

from vehicle_test_suite import AutoTestTimeoutException
from vehicle_test_suite import NotAchievedException

from test_discovery import TestGroup
from test_discovery import testcase


class FenceMargin(TestGroup):
    @testcase(bucket="CopterTests2c")
    def FenceMargin(self, timeout=180):
        '''Test warning on crossing fence margin'''
        # enable fence, disable avoidance
        # margin must be large enough that the circle margin ring
        # (FENCE_RADIUS-FENCE_MARGIN from home) is reached before the
        # polygon edge (~117m from home on this test's track) so both
        # margins are breached simultaneously before any fence is: the
        # combined "Circle and Polygon fences in ..." warning this test
        # expects is only emitted while both margin breaches are active,
        # and an actual fence breach clears that fence's margin state
        self.set_parameters({
            "FENCE_ENABLE": 1,
            "FENCE_TYPE": 6,    # polygon and circle fences
            "FENCE_MARGIN" : 45,
            "FENCE_RADIUS" : 150,
            "AVOID_ENABLE": 0,
            "FENCE_OPTIONS": 4
        })

        self.change_mode("LOITER")
        self.wait_ready_to_arm()

        # fence requires home to be set:
        m = self.poll_home_position(quiet=False)

        # 110m polyfence
        home_loc = self.get_location()
        radius = self.get_parameter("FENCE_RADIUS")
        if self.use_map and self.mavproxy is not None:
            self.mavproxy.send("map circle %f %f %f green\n" % (home_loc.lat, home_loc.lng, radius))

        locs = [
            self.offset_location_ne(home_loc, -110, -110),
            self.offset_location_ne(home_loc, 110, -110),
            self.offset_location_ne(home_loc, 110, 110),
            self.offset_location_ne(home_loc, -110, 110),
        ]
        self.upload_fences_from_locations([
            (mavutil.mavlink.MAV_CMD_NAV_FENCE_POLYGON_VERTEX_INCLUSION, locs),
        ])

        self.takeoff(10, mode="LOITER")

        # first east
        self.progress("turn east")
        self.set_rc(4, 1580)
        self.wait_heading(160, timeout=60)
        self.set_rc(4, 1500)

        fence_radius = self.get_parameter("FENCE_RADIUS")

        self.progress("flying forward (east) until we hit fence")
        pitching_forward = True
        self.set_rc(2, 1100)
        self.wait_statustext("Polygon fence in ([0-9]+[.])?[0-9]?m", regex=True)

        self.wait_statustext("Circle and Polygon fences in ([0-9]+[.])?[0-9]?m", regex=True)
        self.progress("Waiting for fence breach")
        tstart = self.get_sim_time()
        while not self.mode_is("RTL", poll=False):
            if self.get_sim_time_cached() - tstart > 30:
                raise NotAchievedException("Did not breach fence")

            m = self.assert_receive_message('GLOBAL_POSITION_INT')
            alt = m.relative_alt / 1000.0 # mm -> m
            home_distance = self.distance_to_home(use_cached_home=True)
            self.progress("Alt: %.02f  HomeDistance: %.02f (fence radius=%f)" %
                          (alt, home_distance, fence_radius))

        self.wait_statustext("Circle fence cleared margin breach")
        self.progress("Waiting until we get home and disarm")
        tstart = self.get_sim_time()
        while self.get_sim_time_cached() < tstart + timeout:
            m = self.assert_receive_message('GLOBAL_POSITION_INT')
            alt = m.relative_alt / 1000.0 # mm -> m
            home_distance = self.distance_to_home(use_cached_home=True)
            self.progress("Alt: %.02f  HomeDistance: %.02f" %
                          (alt, home_distance))
            # recenter pitch sticks once we're home so we don't fly off again
            if pitching_forward and home_distance < 50:
                pitching_forward = False
                self.set_rc(2, 1475)
                # disable fence
                self.set_parameter("FENCE_ENABLE", 0)
            if (alt <= 1 and home_distance < 10) or (not self.armed() and home_distance < 10):
                # reduce throttle
                self.zero_throttle()
                self.change_mode("LAND")
                self.wait_landed_and_disarmed()
                self.progress("Reached home OK")
                self.zero_throttle()
                return

        # give we're testing RTL, doing one here probably doesn't make sense
        home_distance = self.distance_to_home(use_cached_home=True)
        raise AutoTestTimeoutException(
            "Fence test failed to reach home (%fm distance) - "
            "timed out after %u seconds" % (home_distance, timeout,))
