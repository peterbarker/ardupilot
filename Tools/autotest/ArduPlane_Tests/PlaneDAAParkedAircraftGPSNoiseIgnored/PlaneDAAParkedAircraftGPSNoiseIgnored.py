'''
Plane autotest: PlaneDAAParkedAircraftGPSNoiseIgnored

AP_FLAKE8_CLEAN
'''

from pymavlink import mavutil

from vehicle_test_suite import AltFrame

from test_discovery import TestGroup
from test_discovery import testcase


class PlaneDAAParkedAircraftGPSNoiseIgnored(TestGroup):
    @testcase(bucket="PlaneTests1b")
    def PlaneDAAParkedAircraftGPSNoiseIgnored(self):
        '''is_parked()'s exclusion must survive a single, isolated noisy "moving" sample
        from an otherwise genuinely-parked contact - confirmed live: T2Cruza
        (00000409.BIN, 2026-10-08) briefly avoided a parked Silverback (MAV_SYSID 30, no
        ADS-B) because its direct MAVLink GLOBAL_POSITION_INT reported a momentary
        nonzero ground velocity while its own GPS fix was still acquiring/settling - raw
        GPS noise, not real motion, with none of the transponder-side filtering a real
        ADS-B squawk often has. AVD_GND_DEB requires a fast reading to persist, not just
        one sample, before the parked exclusion is withdrawn.

        One contact, injected continuously; every 10th sample reports 5 m/s instead of
        0 - isolated single-sample spikes, never two in a row, so they never accumulate a
        continuous AVD_GND_DEB-long fast streak. Must never trigger any reaction, same
        assertion as PlaneDAAParkedAircraftIgnored.

        Placed 400 m out on the direct leg, not just 30 m - close to home the vehicle
        is already past the contact (behind it) by the time it is established on the
        leg, climbing out, so a spike there would never have a chance to matter
        regardless of AVD_GND_DEB - see PlaneDAAParkedAircraftGPSNoiseDebounceMatters,
        which proves a spike WOULD matter here if AVD_GND_DEB did not suppress it.'''
        self.install_planedaa_scripts()

        self.set_parameters({
            "SCR_ENABLE": 1,
            "SCR_VM_I_COUNT": 1000000,
            "ADSB_TYPE": 1,
            "AVD_ENABLE": 1,
            "AVD_GND_ALT": 3,
            "AVD_GND_SPD": 2,
            "AVD_GND_DEB": 2,
        })

        self.context_collect('STATUSTEXT')
        self.reboot_sitl()
        self.wait_ready_to_arm()
        self.set_parameter("DAA_TRAP_ACT", 1)  # non-default, so the "TRAPPED" check below has teeth

        home = self.home_position_as_location()
        parked_loc = self.offset_location_ne(home, 400, 0)
        parked_icao = 0xF00093

        self.start_flying_simple_relhome_mission([
            (mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 50),
            (mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 500, 0, 80),
            (mavutil.mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH, 0, 0, 0),
        ])
        self.wait_current_waypoint(2, timeout=120)
        self.wait_text("Plane DAA", check_context=True, timeout=60)

        sample_count = [0]

        def inject():
            sample_count[0] += 1
            noisy = (sample_count[0] % 10 == 0)
            # see PlaneDAAParkedAircraftIgnored's inject() for why this jitters
            self.mav.mav.adsb_vehicle_send(
                parked_icao,
                int(parked_loc.lat * 1e7) + (sample_count[0] % 2),
                int(parked_loc.lng * 1e7),
                mavutil.mavlink.ADSB_ALTITUDE_TYPE_PRESSURE_QNH,
                int(home.get_alt_m(AltFrame.ABSOLUTE) * 1000),
                0,
                500 if noisy else 0,   # 5 m/s horizontal "velocity" on the noisy sample only
                0,
                "PARKED04".encode("ascii"),
                mavutil.mavlink.ADSB_EMITTER_TYPE_SMALL,
                1, 65535, 1200,
            )

        self.daa_watch_no_reaction(60, [
            "AVOIDING", "LOITERING", "LOITER AIRCRAFT", "TRAPPED",
            "Near Miss", "Loss of Well Clear",
        ], inject)

        self.wait_current_waypoint(3, timeout=400)
        self.disarm_vehicle(force=True)
