'''
Plane autotest: PlaneDAALoiterTurnDirection

AP_FLAKE8_CLEAN
'''

import math

from pymavlink import mavutil

from vehicle_test_suite import AltFrame
from vehicle_test_suite import NotAchievedException

from test_discovery import TestGroup
from test_discovery import testcase


class PlaneDAALoiterTurnDirection(TestGroup):
    @testcase(bucket="PlaneTests1b")
    def PlaneDAALoiterTurnDirection(self):
        '''A LEFT-announced aircraft loiter must actually orbit counter-clockwise, not
        clockwise.  daaltr.lua's self.start() hardcoded DO_REPOSITION's yaw (param4) to 0
        regardless of which side the centre was offset to - Plane's
        handle_command_int_do_reposition() reads param4, not the radius sign, to choose
        orbit direction (0/NaN = clockwise, nonzero = counter-clockwise), so every loiter
        flew clockwise even when announced "LOITER left" and the centre was genuinely
        offset to the left.  Confirmed live: an announced "LOITER left" swept +618 deg
        clockwise about its own centre.

        The default DAA_LTR_SIDE quadrant (60 deg, dead-ahead-to-60-right-of-ahead)
        chooses LEFT; a contact placed 30 deg right of a due-north track lands inside
        that quadrant.  Once "LOITERING to" fires, sample ATTITUDE.yaw for a few seconds
        and sum the signed heading deltas (wrapped to +-180 each step): a left/CCW orbit
        nets negative, right/CW nets positive.'''
        self.install_planedaa_scripts()

        self.set_parameters({
            "SCR_ENABLE": 1,
            "SCR_VM_I_COUNT": 1000000,
            "ADSB_TYPE": 1,     # MAVLink: ingest ADSB_VEHICLE with no ADS-B hardware
            "AVD_ENABLE": 1,
            "AVD_WCLR_XY": 200,
            "AVD_WCLR_Z": 50,
        })

        self.context_collect('STATUSTEXT')
        self.reboot_sitl()
        self.wait_ready_to_arm()

        self.set_parameters({
            "DAA_MARGIN_CA": 50,
            "DAA_MARGIN_CA_Z": 30,
            "DAA_AVD_ALT": 50,
            "DAA_AVD_ALT_TP": 1,     # above home - keep the terrain frame out of this
            "DAA_LTR_SIDE": 60,      # default: quadrant dead-ahead to 60 deg right -> LEFT
        })

        icao = 0xA5A5AB

        def inject_aircraft():
            # 30 deg right of the due-north mission track, comfortably inside the
            # default 60 deg DAA_LTR_SIDE quadrant, well inside AVD_WCLR_XY (200 m)
            here = self.get_location()
            contact = self.offset_location_ne(here, 130, 75)   # bearing ~30 deg, ~150 m
            self.mav.mav.adsb_vehicle_send(
                icao,
                int(contact.lat * 1e7),
                int(contact.lng * 1e7),
                mavutil.mavlink.ADSB_ALTITUDE_TYPE_PRESSURE_QNH,
                int(here.get_alt_m(AltFrame.ABSOLUTE) * 1000 + 10 * 1000),
                0, 0, 0,
                "GAJET07".encode("ascii"),
                mavutil.mavlink.ADSB_EMITTER_TYPE_LIGHT,   # crewed -> loiter path
                1, 65535, 1200,
            )

        self.start_flying_simple_relhome_mission([
            (mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 60),
            (mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 3000, 0, 80),
            (mavutil.mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH, 0, 0, 0),
        ])
        self.wait_current_waypoint(2, timeout=120)
        self.wait_text("Plane DAA", check_context=True, timeout=60)

        tstart = self.get_sim_time()
        loitering = False
        while self.get_sim_time() - tstart < 60 and not loitering:
            inject_aircraft()
            self.wait_heartbeat()
            loitering = self.statustext_in_collections("LOITERING to") is not None
        if not loitering:
            raise NotAchievedException("the aircraft loiter never engaged")
        if self.statustext_in_collections("LOITER left") is None:
            raise NotAchievedException(
                "loitering started but never announced LOITER left - did the default "
                "DAA_LTR_SIDE quadrant not pick the left side as expected?")

        # Let the vehicle actually close on and establish on the loiter circle before
        # sampling - the turn immediately after "LOITERING to" is the APPROACH onto the
        # circle (whichever way is the shorter turn onto the new heading), which is not
        # the orbit direction itself and can be either sense regardless of the bug.
        tstart = self.get_sim_time()
        while self.get_sim_time() - tstart < 25:
            inject_aircraft()
            self.wait_heartbeat()

        # Now sample yaw for a while and sum the signed per-step heading change. Keep
        # injecting the contact so the loiter does not get released mid-sample.
        net_deg = 0.0
        last_yaw_deg = None
        tstart = self.get_sim_time()
        while self.get_sim_time() - tstart < 20:
            inject_aircraft()
            m = self.mav.recv_match(type='ATTITUDE', blocking=True, timeout=2)
            if m is None:
                continue
            yaw_deg = math.degrees(m.yaw)
            if last_yaw_deg is not None:
                delta_deg = yaw_deg - last_yaw_deg
                delta_deg = (delta_deg + 180.0) % 360.0 - 180.0
                net_deg += delta_deg
            last_yaw_deg = yaw_deg

        self.progress("net heading change over sample window: %.1f deg" % net_deg)
        if net_deg >= 0:
            raise NotAchievedException(
                "announced LOITER left but net heading change was %.1f deg (clockwise) - "
                "the orbit direction does not match the announced side" % net_deg)
        self.disarm_vehicle(force=True)
