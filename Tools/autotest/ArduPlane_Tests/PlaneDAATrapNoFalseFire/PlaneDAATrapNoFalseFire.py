'''
Plane autotest: PlaneDAATrapNoFalseFire

AP_FLAKE8_CLEAN
'''

from pymavlink import mavutil

from vehicle_test_suite import NotAchievedException

from test_discovery import TestGroup
from test_discovery import testcase


class PlaneDAATrapNoFalseFire(TestGroup):
    @testcase(bucket="PlaneTests1b")
    def PlaneDAATrapNoFalseFire(self):
        '''The trapped-failsafe (DAA_TRAP_ACT) must NOT fire during normal, successful
        avoidance.  An exclusion circle sits across the path to a reachable waypoint;
        the plane detours around it and reaches the waypoint.  With the failsafe
        enabled (RTL, which would be obvious if it wrongly fired), the detour must
        complete WITHOUT tripping the failsafe (no "TRAPPED", mission still runs).'''
        self.install_planedaa_scripts()
        self.set_parameters({
            "SCR_ENABLE": 1,
            "SCR_VM_I_COUNT": 1000000,
            "AVD_ENABLE": 1,
            "FENCE_ENABLE": 0,
            "FENCE_ACTION": 0,
            "FENCE_TYPE": 4,
        })
        home = self.home_position_as_location()
        excl = [(
            mavutil.mavlink.MAV_CMD_NAV_FENCE_CIRCLE_EXCLUSION,
            {"radius": 200, "loc": self.offset_location_ne(home, 1200, 0)},
        )]
        self.upload_fences_from_locations(excl)
        self.context_collect('STATUSTEXT')
        self.reboot_sitl()
        self.wait_ready_to_arm()
        self.set_parameter("DAA_MARGIN_FENCE", 100)
        self.set_parameter("DAA_TRAP_ACT", 1)   # RTL - obvious if it wrongly fired
        self.set_parameter("DAA_TRAP_S", 5)
        self.start_flying_simple_relhome_mission([
            (mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 50),
            (mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 2000, 0, 80),
            (mavutil.mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH, 0, 0, 0),
        ])
        self.wait_current_waypoint(2, timeout=120)
        self.do_fence_enable()
        self.wait_text("Plane DAA", check_context=True, timeout=60)
        # the normal detour must reach the waypoint without the failsafe firing
        self.wait_current_waypoint(3, timeout=400)
        if self.statustext_in_collections("TRAPPED") is not None:
            raise NotAchievedException("trapped-failsafe wrongly fired during normal avoidance")
        # the no-false-fire check above is done; disable the failsafe and the fence for
        # the return-home leg so the non-avoiding flaps.txt landing (which flies back
        # through the exclusion circle) doesn't correctly trip it and fight the landing
        self.set_parameter("DAA_TRAP_ACT", 0)
        self.do_fence_disable()
        self.fly_home_land_and_disarm()
        self.do_fence_disable()
        self.disarm_vehicle(force=True)
