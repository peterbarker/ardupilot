'''
Plane autotest: PlaneDAATrapResetsHung

AP_FLAKE8_CLEAN
'''

from pymavlink import mavutil

from vehicle_test_suite import AltFrame
from vehicle_test_suite import NotAchievedException

from test_discovery import TestGroup
from test_discovery import testcase


class PlaneDAATrapResetsHung(TestGroup):
    @testcase(bucket="PlaneTests1b")
    def PlaneDAATrapResetsHung(self):
        """A moving-obstacle trap must not count its own diversion as hung avoidance."""
        for late in (False, True, "latched"):
            self.start_subtest("moving trap with HUNG latched" if late == "latched" else
                               "near-expired default HUNG timer" if late else "trap longer than HUNG timeout")
            self.context_push()
            self.install_planedaa_scripts()
            if late:
                # Exercise the default 60s/4s boundary deterministically. Seed an
                # almost-expired progress tracker, not the HUNG result, at trap entry.
                with open(self.installed_script_path("planedaa.lua")) as f:
                    content = f.read()
                anchor = "    local function trap_engage(mode_now, cause)\n"
                if content.count(anchor) != 1:
                    raise NotAchievedException("DAA trap timer injection point changed")
                injection = '''        if cause == "moving" then
            hung_best_m = math.max(0, current_loc:get_distance(navigation_target_loc) - 200)
            hung_target_loc = navigation_target_loc:copy()
            hung_since_ms = now_ms - uint32_t(58000)
            %s
            gcs:send_text(6, "DAA test HUNG timer primed")
        end
''' % ("hung_active = true" if late == "latched" else "-- not yet HUNG")
                self.install_script_content("planedaa.lua", content.replace(anchor, anchor + injection))
            self.set_parameters({
                "SCR_ENABLE": 1, "SCR_VM_I_COUNT": 1000000,
                "ADSB_TYPE": 1, "AVD_ENABLE": 1, "FENCE_ENABLE": 0,
                "FENCE_TYPE": 4, "FENCE_ACTION": 0,
                "AVD_WCLR_XY": 200, "AVD_WCLR_Z": 50,
                "AVD_NMAC_XY": 100, "AVD_NMAC_Z": 50,
            })
            self.upload_fences_from_locations([(
                mavutil.mavlink.MAV_CMD_NAV_FENCE_CIRCLE_EXCLUSION,
                {"radius": 300, "loc": self.offset_location_ne(self.home_position_as_location(), 1600, 0)},
            )])
            self.context_collect('STATUSTEXT')
            self.reboot_sitl()
            self.wait_ready_to_arm()
            self.set_parameters({
                "DAA_AVD_ALT": 0, "DAA_TRAP_ACT": 1, "DAA_TRAP_S": 0,
                "DAA_MARGIN_FENCE": 150, "DAA_LKAHD_M": 1000, "DAA_PLAN_M": 1000,
                "DAA_HUNG_ALRT_S": 60 if late else 10,
                "DAA_TRAP_CLR_S": 4 if late else 20,
            })
            self.start_flying_simple_relhome_mission([
                (mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 80),
                (mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 6000, 0, 80),
                (mavutil.mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH, 0, 0, 0),
            ])
            self.wait_current_waypoint(2, timeout=120)
            self.wait_text("Plane DAA", check_context=True, timeout=60)

            def inject_aircraft(distance):
                here = self.get_location()
                contact = self.offset_location_ne(here, 0, distance)
                self.mav.mav.adsb_vehicle_send(
                    0xA5A5B8, int(contact.lat * 1e7), int(contact.lng * 1e7),
                    mavutil.mavlink.ADSB_ALTITUDE_TYPE_PRESSURE_QNH,
                    int(here.get_alt_m(AltFrame.ABSOLUTE) * 1000),
                    0, 0, 0, b"HUNGRESET", mavutil.mavlink.ADSB_EMITTER_TYPE_LIGHT,
                    1, 65535, 1200,
                )

            self.do_fence_enable()
            self.wait_text("AVOIDING", check_context=True, timeout=120)
            tstart = self.get_sim_time()
            while self.get_sim_time() - tstart < 20:
                inject_aircraft(60)
                self.wait_heartbeat()
                if self.mav.flightmode == "RTL":
                    break
            else:
                raise NotAchievedException("moving-obstacle trap did not engage")
            # Move the same contact away immediately: even the 4s release case
            # must resume with a clear traffic picture, not wait for ADS-B ageing.
            inject_aircraft(3000)
            self.do_fence_disable()
            self.wait_text("TRAPPED (moving)", check_context=True, timeout=10)
            if late:
                self.wait_text("DAA test HUNG timer primed", check_context=True, timeout=10)
            self.wait_text("trap clear (moving)", check_context=True, timeout=40)
            self.wait_mode("AUTO", timeout=10)
            self.delay_sim_time(12, reason="verify moving-trap recovery does not re-trap as HUNG")
            if self.mav.flightmode != "AUTO":
                raise NotAchievedException("moving-trap recovery re-trapped into %s" % self.mav.flightmode)
            for text in ("HUNG: no progress", "TRAPPED (hung)", "HUNG clear"):
                if self.statustext_in_collections(text) is not None:
                    raise NotAchievedException("trap diversion incorrectly counted as hung: " + text)
            self.disarm_vehicle(force=True)
            self.context_pop()
