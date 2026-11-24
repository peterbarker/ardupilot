'''
Plane autotest: PlaneDAAFenceAltitude

AP_FLAKE8_CLEAN
'''

from pymavlink import mavutil

from vehicle_test_suite import NotAchievedException

from test_discovery import TestGroup
from test_discovery import testcase


class PlaneDAAFenceAltitude(TestGroup):
    @testcase(bucket="PlaneTests1b")
    def PlaneDAAFenceAltitude(self):
        '''planedaa must avoid the altitude fences FENCE_ALT_MAX (FENCE_TYPE
        bit 0) and FENCE_ALT_MIN (FENCE_TYPE bit 3) in the above-home frame.
        Altitude is a vertical problem that does not fit the horizontal bendy
        ruler, so it is handled by a separate clamp-and-continue path: the
        plane keeps tracking the waypoint horizontally but levels off inside a
        DAA_MARGIN_ALT buffer and must never breach.  Each scenario commands a
        waypoint altitude on the far side of the limit, so the mission alone
        would breach it.  With FENCE_ACTION=0 (report only) any "fence breached"
        statustext fails the test, so a non-breach over a sustained hold proves
        the clamp works.'''
        self._PlaneDAAFenceAltitude(terrain=False)

    @testcase(bucket="PlaneTests1b")
    def PlaneDAAFenceAltitudeTerrain(self):
        '''As PlaneDAAFenceAltitude but with FENCE_ALT_MAX_TP/FENCE_ALT_MIN_TP=3
        (above-terrain frame).  This exercises the script's altitude clamp in
        the terrain frame: detect_altitude_fence()/clamp_alt_to_fence() read and
        clamp altitude AGL via get_alt_m(TERRAIN) using the frame returned by
        get_safe_alt_max()/get_safe_alt_min().  Terrain is served by
        install_terrain_handlers_context() and the band is checked against
        TERRAIN_REPORT.current_height (AGL), so it holds regardless of the
        terrain offset from home.'''
        self._PlaneDAAFenceAltitude(terrain=True)

    def _PlaneDAAFenceAltitude(self, terrain=False):
        self.install_planedaa_scripts()

        # Location::AltFrame: 1 = ABOVE_HOME, 3 = ABOVE_TERRAIN
        alt_tp = 3 if terrain else 1
        # in the terrain frame measure the band against AGL, not height above home
        altitude_source = "TERRAIN_REPORT.current_height" if terrain else None

        if terrain:
            self.install_terrain_handlers_context()

        self.set_parameters({
            "SCR_ENABLE": 1,
            "SCR_VM_I_COUNT": 1000000,
            "AVD_ENABLE": 1,
            "FENCE_ENABLE": 0,    # enabled in-flight so arming is unimpeded
            "FENCE_ACTION": 0,    # report only — a breach must fail the test, not RTL
            "FENCE_MARGIN": 2,    # pin the fence's own margin so the band is deterministic
            "FENCE_ALT_MAX_TP": alt_tp,
            "FENCE_ALT_MIN_TP": alt_tp,
        })
        if terrain:
            self.set_parameter("TERRAIN_ENABLE", 1)

        home = self.home_position_as_location()

        def wait_terrain_ready():
            '''wait until the autopilot has terrain data along the flight path'''
            far = self.offset_location_ne(home, 2600, 0)
            tstart = self.get_sim_time_cached()
            self.progress("Waiting for terrain data along path")
            while True:
                if self.get_sim_time_cached() - tstart > 120:
                    raise NotAchievedException("Did not load required terrain")
                for i in range(11):
                    lat = home.lat + i * (far.lat - home.lat) / 10
                    lon = home.lng + i * (far.lng - home.lng) / 10
                    self.mav.mav.terrain_check_send(int(lat * 1.0e7), int(lon * 1.0e7))
                report = self.assert_receive_message('TERRAIN_REPORT', timeout=60)
                if report.pending == 0:
                    break
                self.delay_sim_time(1, reason="wait for terrain requests")
            self.progress("Terrain ready")

        # With FENCE_MARGIN=2 and DAA_MARGIN_ALT=20 the script clamps to
        # get_safe_alt_max()-20 == (alt_max-2)-20 and get_safe_alt_min()+20 ==
        # (alt_min+2)+20.  For a 120 m ceiling that is ~98 m; for a 60 m floor
        # that is ~82 m.  The "band" brackets that settling altitude while
        # staying on the safe side of the fence.  fence_alt is the parameter we
        # configure for the fence under test; alt_max/alt_min are the actual
        # FENCE_ALT_MAX/MIN values (the unused one is parked well clear).
        scenarios = [
            # max fence at 120 m; mission climbs toward 200 m, clamp ceiling ~98 m
            dict(name="max altitude", fence_type=1, alt_max=120, alt_min=-10,
                 cruise_alt=40, breach_alt=200, band=(90, 119)),
            # min fence at 60 m; mission descends toward 20 m, clamp floor ~82 m.
            # the floor auto-enables once the fence is on and we are above the
            # safe minimum, so we cruise high before commanding the descent.
            dict(name="min altitude", fence_type=8, alt_max=200, alt_min=60,
                 cruise_alt=120, breach_alt=20, band=(61, 90)),
        ]

        for s in scenarios:
            self.start_subtest("planedaa avoids %s fence%s" %
                               (s["name"], " (terrain frame)" if terrain else ""))
            self.context_push()
            self.context_collect('STATUSTEXT')

            self.set_parameters({
                "FENCE_TYPE": s["fence_type"],
                "FENCE_ALT_MAX": s["alt_max"],
                "FENCE_ALT_MIN": s["alt_min"],
            })

            self.reboot_sitl()
            if terrain:
                # reboot clears the terrain cache; reload before relying on AGL
                wait_terrain_ready()
            self.wait_ready_to_arm()

            # DAA parameters only exist once the script has registered them at boot
            self.set_parameter("DAA_MARGIN_ALT", 20)

            # takeoff, a level cruise leg, then the altitude-violating leg.
            # the cruise leg lets planedaa finish its STARTUP_DELAY and become
            # active (with the plane safely clear of the fence) before the
            # climb/descent begins, so there is no startup race with the fence.
            self.start_flying_simple_relhome_mission([
                (mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, s["cruise_alt"]),
                (mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 600, 0, s["cruise_alt"]),
                (mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 2600, 0, s["breach_alt"]),
                (mavutil.mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH, 0, 0, 0),
            ])

            # takeoff complete: now on the level cruise leg, clear of the fence
            self.wait_current_waypoint(2, timeout=180)
            self.do_fence_enable()

            # planedaa announces itself once its STARTUP_DELAY has elapsed
            self.wait_text("Plane DAA", check_context=True, timeout=60)

            # advance onto the leg whose altitude would breach the fence
            self.wait_current_waypoint(3, timeout=120)

            # the plane must move toward the commanded (breaching) altitude
            # but level off inside the safe band rather than crossing.  In
            # the terrain frame the band is checked against AGL.
            self.wait_altitude(s["band"][0], s["band"][1],
                               relative=not terrain,
                               altitude_source=altitude_source,
                               timeout=200)

            # hold while the mission still commands an altitude beyond the
            # fence; the clamp must keep it contained the whole time
            self.delay_sim_time(45, reason="confirm the plane holds inside the fence")

            if self.statustext_in_collections("fence breached") is not None:
                raise NotAchievedException(
                    "Altitude fence breached during %s scenario" % s["name"])
            self.do_fence_disable()
            self.disarm_vehicle(force=True)
            self.context_pop()
