'''
Plane autotest: PlaneDAAStaleLocationExcluded

AP_FLAKE8_CLEAN
'''

from pymavlink import mavutil

from vehicle_test_suite import AltFrame

from test_discovery import TestGroup
from test_discovery import testcase


class PlaneDAAStaleLocationExcluded(TestGroup):
    @testcase(bucket="PlaneTests1b")
    def PlaneDAAStaleLocationExcluded(self):
        '''A direct-MAVLink (GLOBAL_POSITION_INT) contact whose reported lat/lng/alt stop
        changing, even while messages keep arriving, must be excluded from avoidance
        entirely after AVD_LOC_STALE_S - not just from the parked exclusion. A frozen
        feed can carry a wrong altitude or velocity with nothing downstream able to
        tell: this reproduces the live incident (T2Cruza/Silverback, 00000409.BIN /
        00000125.BIN, 2026-10) where `GCS_MAVLink::send_global_position_int()` kept
        resending a stale AHRS location after a transient EKF/GPS hiccup on the sender,
        with no staleness flag in the message.

        AVD_LOC_STALE_S applies ONLY to MAV_COLLISION_SRC_MAVLINK_GPS_GLOBAL_INT - never
        to an ADS-B contact (its timestamp is back-dated by tslc at arrival, which this
        check cannot tell apart from a genuinely frozen feed) and never to a
        POINT_OBSTACLE (deliberately stationary by design, must never be excluded).
        PlaneDAADroneAvoidance is this test's positive control for the direct-MAVLink
        geometry otherwise working; PlaneDAAStaleAdsbNotExcluded below is its negative
        control for the source restriction itself.'''
        self.install_planedaa_scripts()

        self.set_parameters({
            "SCR_ENABLE": 1,
            "SCR_VM_I_COUNT": 1000000,
            "AVD_ENABLE": 1,
            "AVD_UAV_XY": 150,
            "AVD_UAV_Z": 25,
            "AVD_LOC_STALE_S": 2,
        })

        self.context_collect('STATUSTEXT')
        self.reboot_sitl()
        self.wait_ready_to_arm()

        home = self.home_position_as_location()
        drone_loc = self.offset_location_ne(home, 1000, 0)
        drone_sysid = 30  # matches the real Silverback incident's MAV_SYSID

        self.start_flying_simple_relhome_mission([
            (mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 50),
            (mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 2000, 0, 80),
            (mavutil.mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH, 0, 0, 0),
        ])
        self.wait_current_waypoint(2, timeout=120)
        self.wait_text("Plane DAA", check_context=True, timeout=60)

        old_srcSystem = self.mav.mav.srcSystem

        def inject():
            # deliberately NOT jittered, and a fixed altitude rather than tracking the
            # vehicle's own - the point here is a feed that never changes at all. Fixed
            # 90 m above home: close to the 80 m cruise leg (well inside the 25 m gate
            # for most of it), not just the brief climb-out.
            self.mav.mav.srcSystem = drone_sysid
            try:
                self.mav.mav.global_position_int_send(
                    0,
                    int(drone_loc.lat * 1e7),
                    int(drone_loc.lng * 1e7),
                    int(home.get_alt_m(AltFrame.ABSOLUTE) * 1000 + 90000),
                    90000,
                    0, 0, 0,
                    65535,
                )
            finally:
                self.mav.mav.srcSystem = old_srcSystem

        # add_obstacle() trusts a contact's first-ever sample outright -
        # position_update_ms only starts counting from that sample's own arrival
        # time (see its comment in AP_Avoidance.cpp) - so checking for "no
        # reaction" from the very first inject() races AVD_LOC_STALE_S: whether the
        # vehicle has already closed enough to be flagged inside that first window
        # is incidental, not something this test is about. Age the contact well
        # past AVD_LOC_STALE_S here, tolerating any reaction, before the watched
        # window (where a reaction is the test failing) begins.
        stale_s = self.get_parameter("AVD_LOC_STALE_S")
        warmup_tstart = self.get_sim_time()
        while self.get_sim_time() - warmup_tstart < 2 * stale_s:
            inject()
            self.wait_heartbeat()
        # Any ALERT/AVOIDING from the warm-up above is legitimate (the contact was
        # not yet stale) and still sits in the STATUSTEXT collection - clear it so
        # daa_watch_no_reaction()'s very first check isn't failed by a message the
        # watched window itself never produced.
        self.context_clear_collection('STATUSTEXT')

        self.daa_watch_no_reaction(120, [
            "AVOIDING", "LOITERING", "LOITER AIRCRAFT", "TRAPPED",
            "Near Miss", "Loss of Well Clear",
        ], inject)

        self.wait_current_waypoint(3, timeout=400)
        self.disarm_vehicle(force=True)
