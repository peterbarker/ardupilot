'''
Plane autotest: AirspeedEAS2TAS

AP_FLAKE8_CLEAN
'''

from pymavlink import mavutil

from vehicle_test_suite import NotAchievedException

from test_discovery import TestGroup
from test_discovery import testcase


class AirspeedEAS2TAS(TestGroup):
    @testcase(bucket="PlaneTests1a")
    def AirspeedEAS2TAS(self):
        '''check the SITL pitot applies EAS2TAS correctly'''

        self.set_parameters({
            "SIM_WIND_SPD": 0,      # groundspeed must equal true airspeed
            "SIM_WIND_TURB": 0,
        })

        def sample_level_flight(duration=25):
            '''average groundspeed, airspeed and altitude over level flight'''
            sum_gs = 0.0
            sum_as = 0.0
            sum_alt = 0.0
            count = 0
            last_airspeed_message = None
            tstart = self.get_sim_time()
            while self.get_sim_time_cached() - tstart < duration:
                m = self.assert_receive_message('VFR_HUD')
                if abs(m.climb) > 1.0:
                    continue        # only sample steady, level flight
                # pair with the most recent raw airspeed sensor reading,
                # skipping if it hasn't updated since the last sample
                a = self.mav.messages.get('AIRSPEED', None)
                if a is None or a is last_airspeed_message or a.id != 0 or a.airspeed < 1:
                    continue
                last_airspeed_message = a
                sum_gs += m.groundspeed
                sum_as += a.airspeed
                sum_alt += m.alt
                count += 1
            if count < 15:
                raise NotAchievedException("Too few level-flight samples (%u)" % count)
            return sum_gs / count, sum_as / count, sum_alt / count

        def guided_hold_alt(alt_rel_m, timeout=600):
            # loiter at the current position and climb/hold the target altitude
            loc = self.get_location()
            self.run_cmd_int(
                mavutil.mavlink.MAV_CMD_DO_REPOSITION,
                p5=int(loc.lat * 1e7),
                p6=int(loc.lng * 1e7),
                p7=alt_rel_m,
                frame=mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
            )
            self.wait_altitude(alt_rel_m - 5, alt_rel_m + 5, relative=True, timeout=timeout)
            self.delay_sim_time(10, reason="settle into level flight")

        low_alt_rel = 100
        high_alt_rel = 2100
        # Expected change in E2T for a 2000m climb
        e2t_ratio_expected = 1.10
        e2t_ratio_tolerance = 0.03

        self.takeoff(alt=low_alt_rel, mode="TAKEOFF")
        self.change_mode("GUIDED")

        guided_hold_alt(low_alt_rel)
        gs_lo, as_lo, alt_lo = sample_level_flight()

        guided_hold_alt(high_alt_rel)
        gs_hi, as_hi, alt_hi = sample_level_flight()

        e2t_lo = gs_lo / as_lo
        e2t_hi = gs_hi / as_hi
        e2t_ratio = e2t_hi / e2t_lo
        e2t_ratio_min = e2t_ratio_expected - e2t_ratio_tolerance
        e2t_ratio_max = e2t_ratio_expected + e2t_ratio_tolerance

        self.progress("EAS2TAS low:  alt=%.0f gs=%.2f as=%.2f e2t=%.4f" %
                      (alt_lo, gs_lo, as_lo, e2t_lo))
        self.progress("EAS2TAS high: alt=%.0f gs=%.2f as=%.2f e2t=%.4f" %
                      (alt_hi, gs_hi, as_hi, e2t_hi))
        self.progress("EAS2TAS ratio observed=%.4f want=[%.2f, %.2f]" %
                      (e2t_ratio, e2t_ratio_min, e2t_ratio_max))

        if e2t_ratio < e2t_ratio_min or e2t_ratio > e2t_ratio_max:
            raise NotAchievedException(
                f"groundspeed/airspeed ratio {e2t_ratio:.4f} outside "
                f"[{e2t_ratio_min:.2f}, {e2t_ratio_max:.2f}]")

        # force-disarm and reset SITL rather than descending all the way back down
        self.disarm_vehicle(force=True)
        self.reboot_sitl()
