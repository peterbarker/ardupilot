'''
Plane autotest: PlaneDAAStandoffNotDoubleCounted

AP_FLAKE8_CLEAN
'''

from pymavlink import mavutil

from vehicle_test_suite import AltFrame
from vehicle_test_suite import NotAchievedException

from test_discovery import TestGroup
from test_discovery import testcase


class PlaneDAAStandoffNotDoubleCounted(TestGroup):
    @testcase(bucket="PlaneTests1b")
    def PlaneDAAStandoffNotDoubleCounted(self):
        '''The traffic standoff must be AVD_UAV_XY + DAA_MARGIN_UAV, as documented, not
        twice the radius plus the margin.

        AP_Avoidance::distance_to_obstacle() already subtracts the protected radius for the
        emitter type, so the distance handed to Lua is clearance to the edge of the
        protected volume.  find_closest_obstacle() then compared that against
        AVD_UAV_XY + DAA_MARGIN_UAV, adding the radius back, so the real trigger was
        2 x AVD_UAV_XY + DAA_MARGIN_UAV - and it disagreed with find_aircraft(), which
        passes the full standoff against a raw centre distance.

        The drone is parked ahead on the leg with a lateral offset, so the plane closes on
        it and the offset IS the closest approach.  Two things this geometry gets right
        that others do not: the range at which avoidance starts varies by hundreds of
        metres run to run (same order as the effect), and a stationary drone held abeam is
        receding, so assess_obstacle_motion()'s CPA gate correctly drops it in every build.
        Closing on a laterally offset target avoids both.

        With AVD_UAV_XY 300 and DAA_MARGIN_UAV 100 the documented standoff is 400 m and the
        double-counted one is 700 m: a 550 m miss distance must be accepted, and a 250 m one
        must still be avoided - the second case keeps the test honest about drone avoidance
        working at all.'''
        self.install_planedaa_scripts()

        uav_xy_m = 300
        margin_uav_m = 100
        correct_standoff_m = uav_xy_m + margin_uav_m          # 400, documented
        double_counted_m = 2 * uav_xy_m + margin_uav_m        # 700, the bug
        drone_north_m = 3000
        icao = 0xF00099

        self.set_parameters({
            "SCR_ENABLE": 1,
            "SCR_VM_I_COUNT": 1000000,
            "ADSB_TYPE": 1,      # MAVLink: ingest ADSB_VEHICLE with no ADS-B hardware
            "AVD_ENABLE": 1,
            "AVD_UAV_XY": uav_xy_m,
            "AVD_UAV_Z": 25,
        })

        # (lateral miss distance, must it be avoided?)
        scenarios = [
            (550, False),   # outside 400, inside the double-counted 700
            (250, True),    # inside both
        ]

        for (offset_m, want_avoid) in scenarios:
            self.start_subtest("drone passing %u m abeam (avoid=%s)" % (offset_m, want_avoid))
            self.context_push()
            self.context_collect('STATUSTEXT')
            self.reboot_sitl()
            self.wait_ready_to_arm()
            self.set_parameter("DAA_MARGIN_UAV", margin_uav_m)

            home = self.home_position_as_location()
            drone_loc = self.offset_location_ne(home, drone_north_m, offset_m)

            self.start_flying_simple_relhome_mission([
                (mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 50),
                (mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 5000, 0, 80),
                (mavutil.mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH, 0, 0, 0),
            ])
            self.wait_current_waypoint(2, timeout=120)
            self.wait_text("Plane DAA", check_context=True, timeout=60)

            def avoiding_count():
                return len([st for st in self.context_collection('STATUSTEXT')
                            if "AVOIDING" in st.text])

            # fly the leg past the drone, re-sending it because contacts prune after 5 s
            tstart = self.get_sim_time()
            sample_count = [0]
            while self.get_sim_time() - tstart < 200:
                here = self.get_location()
                sample_count[0] += 1
                # see PlaneDAAParkedAircraftIgnored's inject() for why this jitters
                self.mav.mav.adsb_vehicle_send(
                    icao,
                    int(drone_loc.lat * 1e7) + (sample_count[0] % 2),
                    int(drone_loc.lng * 1e7),
                    mavutil.mavlink.ADSB_ALTITUDE_TYPE_PRESSURE_QNH,
                    int(here.get_alt_m(AltFrame.ABSOLUTE) * 1000 + 10000),  # inside AVD_UAV_Z
                    0, 0, 0,
                    "SIMTL99".encode("ascii"),
                    mavutil.mavlink.ADSB_EMITTER_TYPE_UAV,
                    1, 65535, 1200,
                )
                self.wait_heartbeat()
                if want_avoid and avoiding_count() > 0:
                    break
                # stop once we are well past the drone's latitude
                if self.get_distance(home, self.get_location()) > drone_north_m + 400:
                    break

            avoided = avoiding_count() > 0
            self.progress("offset %u m -> avoided=%s (wanted %s)"
                          % (offset_m, avoided, want_avoid))
            if want_avoid and not avoided:
                raise NotAchievedException(
                    "did not avoid a drone passing %u m abeam, inside the documented "
                    "standoff of %u m" % (offset_m, correct_standoff_m))
            if avoided and not want_avoid:
                raise NotAchievedException(
                    "avoided a drone passing %u m abeam, outside the documented standoff "
                    "of %u m (AVD_UAV_XY + DAA_MARGIN_UAV): the radius is being counted "
                    "twice, which reacts out to %u m"
                    % (offset_m, correct_standoff_m, double_counted_m))
            self.disarm_vehicle(force=True)
            self.context_pop()
