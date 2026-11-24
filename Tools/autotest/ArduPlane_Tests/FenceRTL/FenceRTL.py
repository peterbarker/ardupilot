'''
Plane autotest: FenceRTL

AP_FLAKE8_CLEAN
'''

from pymavlink import mavutil

from vehicle_test_suite import AltFrame
from vehicle_test_suite import NotAchievedException

from test_discovery import TestGroup
from test_discovery import testcase


class FenceRTL(TestGroup):
    def test_fence_breach_circle_at(self, loc, disable_on_breach=False):
        self.load_fence("CMAC-fence.txt")
        want_radius = 100
        # when ArduPlane is fixed, remove this fudge factor
        REALLY_BAD_FUDGE_FACTOR = 1.16
        expected_radius = REALLY_BAD_FUDGE_FACTOR * want_radius
        self.set_parameters({
            "RTL_RADIUS": want_radius,
            "NAVL1_LIM_BANK": 60,
            "FENCE_ACTION": 1, # AC_FENCE_ACTION_RTL_AND_LAND == 1. mavutil.mavlink.FENCE_ACTION_RTL == 4
        })

        self.wait_ready_to_arm()  # need an origin to load fence

        self.do_fence_enable()
        self.assert_fence_sys_status(True, True, True)

        self.takeoff(alt=45, alt_max=300)

        tstart = self.get_sim_time()
        while True:
            if self.get_sim_time() - tstart > 30:
                raise NotAchievedException("Did not breach fence")
            m = self.assert_receive_message('FENCE_STATUS', timeout=2)
            if m.breach_status == 0:
                continue

            # we've breached; check our state;
            if m.breach_type != mavutil.mavlink.FENCE_BREACH_BOUNDARY:
                raise NotAchievedException("Unexpected breach type %u" %
                                           (m.breach_type,))
            if m.breach_count == 0:
                raise NotAchievedException("Unexpected breach count %u" %
                                           (m.breach_count,))
            self.assert_fence_sys_status(True, True, False)
            break

        self.wait_circling_point_with_radius(loc, expected_radius)
        self.do_fence_disable()
        self.disarm_vehicle(force=True)
        self.reboot_sitl()

    @testcase(bucket="PlaneTests1b")
    def FenceRTL(self):
        '''Test Fence RTL'''
        self.progress("Testing FENCE_ACTION_RTL no rally point")
        # have to disable the fence once we've breached or we breach
        # it as part of the loiter-at-home!
        self.test_fence_breach_circle_at(self.home_position_as_location())

    @testcase(bucket="PlaneTests1a")
    def FenceRTLRally(self):
        '''Test Fence RTL Rally'''
        self.progress("Testing FENCE_ACTION_RTL with rally point")

        self.wait_ready_to_arm()
        loc = self.offset_location_ne(self.home_position_as_location(), 50, -50)
        loc.set_alt_m(15, AltFrame.ABOVE_HOME)
        self.upload_rally_points_from_locations([loc])
        self.test_fence_breach_circle_at(loc)
