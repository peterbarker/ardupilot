'''
Plane autotest: PlaneDAADroneCrossing

AP_FLAKE8_CLEAN
'''

import math

from pymavlink import mavutil

from vehicle_test_suite import AltFrame
from vehicle_test_suite import NotAchievedException

from test_discovery import TestGroup
from test_discovery import testcase


class PlaneDAADroneCrossing(TestGroup):
    @testcase(bucket="PlaneTests1b")
    def PlaneDAADroneCrossing(self):
        '''planedaa must avoid a *moving* ADS-B drone and reach its waypoint, then
        land cleanly.  A drone slow-overtake-crosses the northbound corridor (it
        tracks north a little slower than the plane and drifts west) with matched
        altitude, so it stays a horizontal threat for tens of seconds.  Unlike
        PlaneDAADroneAvoidance the intruder reports a non-zero velocity, so this is
        the test that drives the moving-obstacle code path -- assess_obstacle_motion()
        (closing/CPA/pass-behind) and refine_avoidance_bearing() (side commitment +
        heading slew limit).  It also applies a loose bound on flown-course reversals;
        see the note at that check -- SITL fixed-wing does not visibly wiggle even
        with the smoothing off, so that bound is a gross-oscillation guard, not a
        proof of the smoothing.'''
        self.install_planedaa_scripts()

        self.set_parameters({
            "SCR_ENABLE": 1,
            "SCR_VM_I_COUNT": 1000000,
            "ADSB_TYPE": 1,     # MAVLink: ingest ADSB_VEHICLE with no ADS-B hardware
            "AVD_ENABLE": 1,    # required for AP_Avoidance to pull ADSB samples
            "AVD_UAV_XY": 150,  # horizontal UAV standoff
            "AVD_UAV_Z": 25,    # vertical gate; the intruder (+10 m) sits inside it
        })

        self.context_collect('STATUSTEXT')
        self.reboot_sitl()
        self.wait_ready_to_arm()

        home = self.home_position_as_location()
        icao = 0xF00099

        # a slow-overtake crossing: the drone tracks north a little slower than
        # the plane, so the plane closes only a few m/s and the conflict lasts
        # tens of seconds, while the drone drifts steadily west across the track.
        # A long, side-ambiguous conflict is what makes a per-cycle bendy ruler
        # flip-flop left/right; the smoothing must instead hold a committed arc.
        drone_north0 = 500.0
        drone_east0 = 70.0
        drone_vn = 15.0             # m/s north (plane cruises faster and overtakes)
        drone_vw = 4.0             # m/s west (lateral drift across the track)
        drone_speed = math.sqrt(drone_vn ** 2 + drone_vw ** 2)
        drone_hdg_deg = math.degrees(math.atan2(-drone_vw, drone_vn)) % 360

        self.start_flying_simple_relhome_mission([
            (mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 50),
            (mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 2000, 0, 80),
            (mavutil.mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH, 0, 0, 0),
        ])
        self.wait_current_waypoint(2, timeout=120)
        self.wait_text("Plane DAA", check_context=True, timeout=60)

        tstart = self.get_sim_time()
        avoided = False
        avoid_start = None
        last_cog_t = 0.0
        cogs = []               # (t, ground_course_deg) sampled while avoiding

        while self.get_sim_time() - tstart < 180:
            now = self.get_sim_time()
            elapsed = now - tstart

            # advance the drone (north + west) and re-inject (obstacles prune after 5 s)
            drone_north = drone_north0 + drone_vn * elapsed
            drone_east = drone_east0 - drone_vw * elapsed
            drone_loc = self.offset_location_ne(home, int(round(drone_north)), int(round(drone_east)))
            here = self.get_location()
            self.mav.mav.adsb_vehicle_send(
                icao,
                int(drone_loc.lat * 1e7),
                int(drone_loc.lng * 1e7),
                mavutil.mavlink.ADSB_ALTITUDE_TYPE_PRESSURE_QNH,
                int(here.get_alt_m(AltFrame.ABSOLUTE) * 1000 + 10000),   # 10 m up, inside the 25 m gate
                int(round(drone_hdg_deg * 100)),  # heading cdeg
                int(round(drone_speed * 100)),    # horizontal velocity cm/s
                0,                              # vertical velocity cm/s
                "SIMXNG".encode("ascii"),
                mavutil.mavlink.ADSB_EMITTER_TYPE_UAV,
                1,      # tslc
                65535,  # flags (position/velocity/heading all valid)
                1200,   # squawk
            )

            # latch the first avoidance from the STATUSTEXT collection (robust
            # against races with the context collector / wait_text)
            if not avoided and self.statustext_in_collections("AVOIDING") is not None:
                avoided = True
                avoid_start = now

            # once avoiding, sample the flown ground course at ~2 Hz
            if avoided and now - last_cog_t >= 0.5:
                gpi = self.mav.recv_match(type='GLOBAL_POSITION_INT',
                                          blocking=True, timeout=1)
                if gpi is not None and (gpi.vx != 0 or gpi.vy != 0):
                    cog = math.degrees(math.atan2(gpi.vy, gpi.vx))
                    cogs.append((now, cog))
                    last_cog_t = now
                # stop collecting once the drone is well past and the plane has
                # cleared the crossing (measured a decent avoidance window)
                if avoid_start is not None and now - avoid_start > 60:
                    break

            self.delay_sim_time(0.25, reason="pace drone injection")

        if not avoided:
            raise NotAchievedException("planedaa did not avoid the crossing drone")
        if len(cogs) < 10:
            raise NotAchievedException(
                "too few ground-course samples (%u) to judge smoothness" % len(cogs))

        # Count direction reversals in the flown ground course.  NOTE: in SITL
        # fixed-wing the flown path stays smooth even with the smoothing disabled
        # (airframe dynamics + L1 wash out the per-cycle bendy-ruler oscillation),
        # so this is a gross-oscillation sanity bound, not a proof of the smoothing
        # -- it catches a future change that makes the aircraft visibly hunt, while
        # the real value of this test is exercising the moving-obstacle code path
        # (assess_obstacle_motion / refine_avoidance_bearing) that the stationary
        # PlaneDAADroneAvoidance never reaches.  A deadband rejects attitude noise.
        # The type-aware CPA standoff (get_standoff) re-decides the crossing conflict
        # from current geometry every cycle with NO hold -- by design, for maximum
        # responsiveness to an unpredictable/manoeuvring drone -- so a marginal slow
        # overtake legitimately reverses more (slew-limited, so bounded/safe).  The
        # bound is therefore loose; it only fires on genuinely wild oscillation.
        deadband_deg = 6.0
        reversals = 0
        prev_sign = 0
        for i in range(1, len(cogs)):
            d = cogs[i][1] - cogs[i - 1][1]
            while d > 180:
                d -= 360
            while d < -180:
                d += 360
            if abs(d) < deadband_deg:
                continue
            sign = 1 if d > 0 else -1
            if prev_sign != 0 and sign != prev_sign:
                reversals += 1
            prev_sign = sign

        self.progress("Crossing-drone avoidance: %u course samples, %u reversals"
                      % (len(cogs), reversals))

        max_reversals = 35
        if reversals > max_reversals:
            raise NotAchievedException(
                "ground course hunted badly: %u reversals > %u"
                % (reversals, max_reversals))

        # the plane must still reach the waypoint (not be trapped by the drone) ...
        self.wait_current_waypoint(3, timeout=400)
        # ... and the test must leave the vehicle landed and disarmed on the ground
        self.fly_home_land_and_disarm()
        self.disarm_vehicle(force=True)
