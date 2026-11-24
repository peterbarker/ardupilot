'''
QuadPlane autotest: QRTLGradualAltDescentTerrain

AP_FLAKE8_CLEAN
'''

from pymavlink import mavutil

import vehicle_test_suite

from vehicle_test_suite import NotAchievedException

from test_discovery import TestGroup
from test_discovery import testcase


class QRTLGradualAltDescentTerrain(TestGroup):
    @testcase(bucket="QuadPlaneTests1c")
    def QRTLGradualAltDescentTerrain(self):
        '''check QRTL's approach altitude ramp is computed in the correct
        altitude frame when QRTL is terrain following'''
        self.install_terrain_handlers_context()

        qrtl_alt = 20
        rtl_altitude = 60
        self.set_parameters({
            "TERRAIN_ENABLE": 1,
            "TERRAIN_FOLLOW": 1 << 9,  # bit 9 is QRTL
            "TERRAIN_OPTIONS": 1 << 2,  # accept the cached terrain data as-is
            "TERRAIN_LOOKAHD": 0,  # isolate the approach ramp; no lookahead confound
            "ALT_SLOPE_MIN": 1,  # ensure the gradual ramp is used, not an immediate jump
            "Q_RTL_ALT": qrtl_alt,
            "RTL_ALTITUDE": rtl_altitude,
            "Q_TRANS_DECEL": 0.6,
        })

        # move home to CMAC: the default QuadPlane test location is very
        # flat, and this test needs real, cached-locally terrain relief
        # to distinguish correct from buggy behaviour
        self.customise_SITL_commandline(["--home", "-35.362938,149.165085,585,354"])

        # fly west, where the ground rises steadily away from home for a
        # couple of kilometres, so the QRTL destination (home) ends up
        # well below the point the approach starts from
        homeloc = self.home_position_as_location()
        far_loc = self.offset_location_ne(homeloc, 0, -2000)

        # prime the terrain database along the route before arming, or
        # the prearm terrain check will never clear
        tstart = self.get_sim_time()
        while True:
            if self.get_sim_time_cached() - tstart > 60:
                raise NotAchievedException("Did not get required terrain data")
            pending = 0
            for i in range(11):
                lat = homeloc.lat + i * (far_loc.lat - homeloc.lat) / 10
                lon = homeloc.lng + i * (far_loc.lng - homeloc.lng) / 10
                self.mav.mav.terrain_check_send(int(lat*1e7), int(lon*1e7))
                report = self.assert_receive_message('TERRAIN_REPORT', timeout=60)
                pending += report.pending
            self.progress("Terrain pending=%u" % pending)
            if pending == 0:
                break

        self.start_flying_simple_relhome_mission([
            (mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 100),
            (mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, -2000, 100),
        ])

        self.wait_current_waypoint(2)
        self.wait_distance_to_home(1500, 1600, timeout=180)

        entry_agl = self.get_altitude(altitude_source="TERRAIN_REPORT.current_height")
        self.progress("Entering QRTL at %.1fm AGL" % entry_agl)
        self.change_mode('QRTL')

        # the approach ramp must start from where the aircraft actually
        # is. If its initial excess-height figure is computed in the
        # wrong altitude frame (absolute rather than terrain-relative)
        # the sloping ground between here and home biases it by the
        # ground-height difference between the two points, and
        # alt_error jumps by roughly that amount the moment QRTL is
        # entered instead of starting near zero
        class MonitorQRTLTerrainEntryError(vehicle_test_suite.TestSuite.MessageHook):
            '''watches NAV_CONTROLLER_OUTPUT.alt_error right after QRTL is
            entered over sloping terrain; a large jump means the approach
            ramp mixed absolute and terrain-relative altitude frames'''
            def __init__(self, suite, max_entry_error=8):
                super(MonitorQRTLTerrainEntryError, self).__init__(suite)
                self.max_entry_error = max_entry_error  # metres
                self.seen_sample = False

            def process(self, mav, m):
                if m.get_type() != 'NAV_CONTROLLER_OUTPUT':
                    return
                self.seen_sample = True
                self.progress("QRTL terrain entry: alt_error=%.1f" % m.alt_error)
                if abs(m.alt_error) > self.max_entry_error:
                    raise NotAchievedException(
                        "QRTL target altitude jumped by %.1fm on entry over "
                        "sloping terrain (max %.1fm); the approach ramp is "
                        "mixing altitude frames" % (m.alt_error, self.max_entry_error))

        self.context_push()
        monitor = MonitorQRTLTerrainEntryError(self)
        self.install_message_hook_context(monitor)
        self.delay_sim_time(5, reason="sample QRTL entry altitude error")
        self.context_pop()

        if not monitor.seen_sample:
            raise NotAchievedException("Never saw a NAV_CONTROLLER_OUTPUT sample")

        # let it continue home, transition and land normally
        self.wait_disarmed(timeout=300)
