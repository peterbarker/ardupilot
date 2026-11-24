'''
Plane autotest: PlaneDAALoiterHomeFenceOnly

AP_FLAKE8_CLEAN
'''

import math

from pymavlink import mavutil

from vehicle_test_suite import AltFrame
from vehicle_test_suite import NotAchievedException

from test_discovery import TestGroup
from test_discovery import testcase


class PlaneDAALoiterHomeFenceOnly(TestGroup):
    @testcase(bucket="PlaneTests1b")
    def PlaneDAALoiterHomeFenceOnly(self):
        '''The aircraft loiter's fence-clearance check must see a home-centred radius
        fence (FENCE_RADIUS/FENCE_TYPE bit AC_FENCE_TYPE_CIRCLE), not just loaded polygon
        fences. AP_OAScripting::fence_distance() used to walk the polyfence loader's raw
        boundary arrays directly, which have no representation of FENCE_RADIUS at all (it
        lives only as AC_Fence's own _circle_radius_m) - so a home-circle-only setup left
        nearest_fence_clearance_m() always nil, and the loiter proceeded on its default
        side with no fence awareness whatsoever. Confirmed live: a loiter reached 1064 m
        from home against a 1000 m fence this way.

        Same geometry idea as PlaneDAALoiterFencePicksClearSide, but with ONLY a home
        circle active (no polygon fence loaded) and sized so BOTH loiter-side offsets
        breach it - the mission track runs due north from home, so the left and right
        WP_LOITER_RAD-abeam candidate centres are (by symmetry) almost exactly
        equidistant from home, and so is the fence's signed clearance to each. With the
        fix, this must refuse ("no fence-clear side"), never announce a flip, and never
        start the loiter. Pre-fix, this fence is invisible to the check and the loiter
        just proceeds as if no fence existed at all.'''
        self.install_planedaa_scripts()

        self.set_parameters({
            "SCR_ENABLE": 1,
            "SCR_VM_I_COUNT": 1000000,
            "ADSB_TYPE": 1,      # MAVLink: ingest ADSB_VEHICLE with no ADS-B hardware
            "AVD_ENABLE": 1,
            "AVD_WCLR_XY": 200,
            "AVD_WCLR_Z": 50,
            "FENCE_ENABLE": 0,   # enabled in flight so arming is unimpeded
            "FENCE_ACTION": 0,   # report only
            "FENCE_TYPE": 2,     # home circle ONLY - no polygon bit set
            "WP_LOITER_RAD": 400,
        })

        self.context_collect('STATUSTEXT')
        self.reboot_sitl()
        self.wait_ready_to_arm()

        self.set_parameters({
            "DAA_MARGIN_CA": 50,
            "DAA_MARGIN_CA_Z": 30,
            "DAA_AVD_ALT": 50,
            "DAA_AVD_ALT_TP": 1,     # above home - keep the terrain frame out of this
            "DAA_MARGIN_FENCE": 20,
        })

        icao = 0xA5A5AC

        def inject_aircraft():
            here = self.get_location()
            contact = self.offset_location_ne(here, 0, -120)
            self.mav.mav.adsb_vehicle_send(
                icao,
                int(contact.lat * 1e7),
                int(contact.lng * 1e7),
                mavutil.mavlink.ADSB_ALTITUDE_TYPE_PRESSURE_QNH,
                int(here.get_alt_m(AltFrame.ABSOLUTE) * 1000 + 10 * 1000),
                0, 0, 0,
                "GAJET08".encode("ascii"),
                mavutil.mavlink.ADSB_EMITTER_TYPE_LIGHT,   # crewed -> loiter path
                1, 65535, 1200,
            )

        self.start_flying_simple_relhome_mission([
            (mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 60),
            (mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 3000, 0, 80),
            (mavutil.mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH, 0, 0, 0),
        ])
        self.do_fence_enable()
        self.wait_text("Plane DAA", check_context=True, timeout=60)

        # Size the home circle off the vehicle's REAL distance from home right now, not a
        # guessed figure: "current waypoint is 2" (as PlaneDAALoiterFencePicksClearSide
        # waits for) means "now navigating toward WP2", not "arrived at WP2's location" -
        # the vehicle can still be close to home at that point.
        #
        # The loiter CENTRE itself (WP_LOITER_RAD abeam of the vehicle) must stay the
        # SAME side of the real fence boundary Plane's own DO_REPOSITION sanitize already
        # checks (handle_command_int_do_reposition()'s check_location_within_fence()) -
        # otherwise that pre-existing core check rejects the centre outright
        # ("set_vehicle FAILED"), which proves nothing about THIS fix (the orbit-aware
        # clearance check, which is about the RADIUS around an otherwise-legal centre, not
        # the centre point itself - see daaltr.lua's own comment on self.start()). So the
        # centre's distance from home must stay just INSIDE FENCE_RADIUS (positive
        # clearance), by less than the (WP_LOITER_RAD + DAA_MARGIN_FENCE) threshold.
        wp_loiter_rad_m = 400
        home = self.home_position_as_location()
        here = self.get_location()
        dist_from_home_m = self.get_distance(home, here)
        # the abeam centre is ~sqrt(dist^2 + WP_LOITER_RAD^2) from home when dist is small
        # relative to WP_LOITER_RAD (current heading ~= the home-to-WP2 radial)
        centre_dist_m = math.sqrt(dist_from_home_m ** 2 + wp_loiter_rad_m ** 2)
        # ~50 m clearance: comfortably positive (core sanitize passes) and well under
        # the 420 m (WP_LOITER_RAD + DAA_MARGIN_FENCE) threshold
        fence_radius_m = centre_dist_m + 50
        self.set_parameter("FENCE_RADIUS", fence_radius_m)
        self.progress("vehicle %.0f m from home, centre ~%.0f m from home - "
                      "FENCE_RADIUS set to %.0f m"
                      % (dist_from_home_m, centre_dist_m, fence_radius_m))

        tstart = self.get_sim_time()
        decided = False
        while self.get_sim_time() - tstart < 60 and not decided:
            inject_aircraft()
            self.wait_heartbeat()
            decided = (self.statustext_in_collections("LOITERING to") is not None
                       or self.statustext_in_collections("no fence-clear side") is not None)

        loitering = self.statustext_in_collections("LOITERING to") is not None
        no_clear_side = self.statustext_in_collections("no fence-clear side") is not None
        self.progress("loitering=%s, no_clear_side=%s" % (loitering, no_clear_side))

        if loitering:
            raise NotAchievedException(
                "the loiter started even though only a home-circle fence is active and "
                "both sides should breach it - fence_distance() still cannot see FENCE_RADIUS")
        if not no_clear_side:
            raise NotAchievedException(
                "neither started the loiter nor announced a refusal - the contact was "
                "never detected, so this test proves nothing")
        self.disarm_vehicle(force=True)
