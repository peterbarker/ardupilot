'''
Plane autotest: PlaneDAAFenceCandidateChecksActiveTraffic

AP_FLAKE8_CLEAN
'''

from pymavlink import mavutil

from vehicle_test_suite import AltFrame
from vehicle_test_suite import NotAchievedException

from test_discovery import TestGroup
from test_discovery import testcase


class PlaneDAAFenceCandidateChecksActiveTraffic(TestGroup):
    @testcase(bucket="PlaneTests1b")
    def PlaneDAAFenceCandidateChecksActiveTraffic(self):
        '''validate_horizontal_release() also runs every cycle a moving obstacle is
        STILL the live winner (not dismissed), so a fence appearing nearby can replace
        an ongoing drone avoidance with a fence-driven bearing instead. That candidate
        bearing was only ever checked against fences, never against the drone still
        being avoided - unlike the dismissed-contact case next door
        (PlaneDAAMaskedDroneBehindDismissedAircraft), nothing stopped it from swinging
        back into the drone's own keep-out.

        A drone sits fixed off one side of a long straight leg, close enough that
        avoiding it is unavoidable and sustained (never opens past well-clear, so it is
        never dismissed - DAAR.Gon stays 0 throughout). A thin exclusion strip sits
        along the real lateral path that detour takes, tight enough (DAA_MARGIN_FENCE=0,
        the real turn-radius fallback) that the fence-driven candidate has little room
        to spare.

        Asserted via the dataflash DAAR log: on every cycle the drone was the
        pre-resolver pick (Sel==UNCREWED_AIRCRAFT) and still live (Gon==0) and a fence
        candidate was adopted (Held==1), CmdD - the clearance of the path actually
        being commanded, which find_closest_obstacle() computes against every obstacle
        type, not just fences - must never show a real breach of the drone.

        WP2's altitude (200m) is deliberately far above the cruise altitude the
        vehicle is actually at while avoiding the drone (climbing out from 60m, still
        well under 100m over this geometry's drone/fence window) - more than
        AVD_UAV_Z apart. masked_contact_on_bearing()'s probe locations are built from
        target_loc (WP2), so if they adopt its altitude outright instead of the
        vehicle's own current altitude (the bug this test exists to catch -
        location_for_candidate() and location_project() both do that unless
        corrected, the same near-end flattening probe_bearing() already guards
        against with copy_alt_from()/interpolate_alt()), the drone - injected at the
        vehicle's own live altitude every cycle - reads as more than AVD_UAV_Z below
        the probe and is never flagged as a conflict, and the fence candidate is
        adopted straight through it. This is a weak negative control: reverting just
        the copy_alt_from()/interpolate_alt() calls in masked_contact_on_bearing()
        does make it fail, but only a small fraction of repeated runs, and with a
        small CmdD breach - the climb-out itself crosses the exclusion strip before
        the deliberate drone-avoidance detour, so most of the discriminating signal
        this test wants is masked by that incidental breach.

        CmdD (find_closest_obstacle() over every obstacle type) is only a drone
        check while the fence itself stays clear: enabling the fence in-flight,
        after the drone-avoidance turn was already under way, could itself land the
        vehicle inside the exclusion strip before the fence ever engages, making
        CmdD<0 reflect that pre-existing fence breach (ObjT==FENCE_POLYGON_EXCLUSION)
        and not the drone at all. The fence is enabled here before the mission
        starts - home is outside the strip, so arming is unaffected - and the
        breach check only trusts CmdD<0 on a cycle where the separately-logged
        fence-only clearance (FPrD) is NOT itself breached, so a real fence breach
        can never be misread as a drone breach.'''
        self.install_planedaa_scripts()

        self.set_parameters({
            "SCR_ENABLE": 1,
            "SCR_VM_I_COUNT": 1000000,
            "ADSB_TYPE": 1,     # MAVLink: ingest ADSB_VEHICLE with no ADS-B hardware
            "AVD_ENABLE": 1,
            "AVD_UAV_XY": 100,
            "AVD_UAV_Z": 25,
            "FENCE_ENABLE": 0,      # set by do_fence_enable() below, before the mission starts
            "FENCE_TYPE": 4,        # polyfence (circle exclusion)
            "FENCE_ACTION": 0,      # report only - a breach must fail the test, not RTL
            "ROLL_LIMIT_DEG": 65,
            "AIRSPEED_CRUISE": 25,
            "RLL2SRV_RMAX": 0,
            "RLL2SRV_TCONST": 0.5,
        })

        home = self.home_position_as_location()
        # Fixed 60 m east of the direct home->WP2 line, close enough with AVD_UAV_XY +
        # DAA_MARGIN_UAV below (130 m total) that avoiding it needs a real detour to the
        # west - a detour is unavoidable, and never opens past well-clear (fixed, zero
        # velocity, vehicle closes on it the whole leg) so it is never dismissed.
        drone_loc = self.offset_location_ne(home, 500, 60)
        # A thin exclusion strip along the west side of the direct line, spanning the
        # real lateral deviation this geometry's avoidance turn produces (measured from
        # a prior run's DAAR/POS log: ~10-22 m west of the line over roughly N 150-350) -
        # not a shape derived from the drone's own keep-out radius, which is a dynamic
        # CPA threshold, not a static area the flown path avoids wholesale.
        self.upload_fences_from_locations([
            (mavutil.mavlink.MAV_CMD_NAV_FENCE_POLYGON_VERTEX_EXCLUSION, [
                self.offset_location_ne(home, 150, -4),
                self.offset_location_ne(home, 350, -4),
                self.offset_location_ne(home, 350, -32),
                self.offset_location_ne(home, 150, -32),
            ]),
        ])
        drone_icao = 0xD3D3D3

        self.context_collect('STATUSTEXT')
        self.reboot_sitl()
        self.wait_ready_to_arm()
        self.set_parameters({
            "DAA_MARGIN_UAV": 30,      # drone standoff = 130 m total with AVD_UAV_XY
            "DAA_MARGIN_FENCE": 0,
            "DAA_PLAN_M": 200,
            "DAA_LKAHD_M": 500,
        })
        # Enabled here, before the mission starts - home is outside the exclusion
        # strip, so this does not block arming. Enabling it only once the vehicle
        # was already turning to avoid the drone let it reach the strip before the
        # fence ever engaged, landing it already inside the exclusion area the
        # moment the fence turned on - a breach with nothing to do with this test.
        self.do_fence_enable()

        drone_sample_count = [0]

        def inject_drone():
            here = self.get_location()
            drone_sample_count[0] += 1
            # see PlaneDAAParkedAircraftIgnored's inject() for why this jitters
            self.mav.mav.adsb_vehicle_send(
                drone_icao,
                int(drone_loc.lat * 1e7) + (drone_sample_count[0] % 2),
                int(drone_loc.lng * 1e7),
                mavutil.mavlink.ADSB_ALTITUDE_TYPE_PRESSURE_QNH,
                int(here.get_alt_m(AltFrame.ABSOLUTE) * 1000 + 10000),
                0, 0, 0,
                "ACTVDR1".encode("ascii"),
                mavutil.mavlink.ADSB_EMITTER_TYPE_UAV,     # -> AVD_UAV_XY radius
                1, 65535, 1200,
            )

        self.start_flying_simple_relhome_mission([
            (mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 60),
            (mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 1000, 0, 200),
            (mavutil.mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH, 0, 0, 0),
        ])
        self.wait_current_waypoint(2, timeout=120)
        self.wait_text("Plane DAA", check_context=True, timeout=60)

        tstart = self.get_sim_time()
        while self.get_sim_time() - tstart < 90:
            inject_drone()
            self.wait_heartbeat()

        log_filepath = self.current_onboard_log_filepath()
        self.disarm_vehicle(force=True)

        self.progress("Inspecting DFReader for a fence candidate breaching the active drone")
        dfreader = self.dfreader_for_path(log_filepath)
        OBSTACLE_TYPE_UNCREWED_AIRCRAFT = 1
        saw_fence_override_of_active_drone = False
        breach_m = None
        while True:
            m = dfreader.recv_match(type=['DAAR'])
            if m is None:
                break
            if (m.Sel == OBSTACLE_TYPE_UNCREWED_AIRCRAFT and m.Gon == 0 and m.Held == 1 and
                    m.FPrD >= -1.0):
                saw_fence_override_of_active_drone = True
                # FPrD is the same candidate path's fence-only clearance (also
                # signed, positive clear): only trust CmdD<0 as a DRONE breach when
                # the fence itself was not also breached that cycle, or a real fence
                # breach (ObjT==FENCE_POLYGON_EXCLUSION winning find_closest_obstacle())
                # reads as a false drone breach here.
                if m.CmdD < -1.0 and (breach_m is None or m.CmdD < breach_m):
                    breach_m = m.CmdD
        if not saw_fence_override_of_active_drone:
            raise NotAchievedException(
                "never saw a fence-clear DAAR.Sel==UNCREWED_AIRCRAFT with Gon==0 and Held==1 - the fence "
                "candidate never overrode an actively-avoided (non-dismissed) drone, "
                "geometry did not reproduce")
        if breach_m is not None:
            raise NotAchievedException(
                "a fence-driven candidate crossed %.1fm into the still-active drone's "
                "own keep-out (DAAR.CmdD)" % (-breach_m,))
