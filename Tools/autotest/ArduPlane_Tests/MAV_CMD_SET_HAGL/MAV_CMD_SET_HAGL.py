'''
Plane autotest: MAV_CMD_SET_HAGL

AP_FLAKE8_CLEAN
'''

from pymavlink import mavutil

from vehicle_test_suite import NotAchievedException

from test_discovery import TestGroup
from test_discovery import testcase


class MAV_CMD_SET_HAGL(TestGroup):
    @testcase(bucket="PlaneTests1b")
    def MAV_CMD_SET_HAGL(self):
        '''test height above ground from MAV_CMD_SET_HAGL is used for the landing flare'''
        self.start_flying_simple_relhome_mission([
            (mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 30),
            (mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 1000, 0, 30),
            (mavutil.mavlink.MAV_CMD_NAV_LAND, 10, 0, 0),
        ])
        self.wait_current_waypoint(3, timeout=120)

        self.context_collect('STATUSTEXT')
        tstart = self.get_sim_time()
        flare = None
        while flare is None:
            if self.get_sim_time_cached() - tstart > 120:
                raise NotAchievedException("Did not flare")
            # tell the vehicle it is just above the ground while it is still well up the approach:
            self.run_cmd_int(
                mavutil.mavlink.MAV_CMD_SET_HAGL,
                p1=0.5,  # height above ground (m)
                p3=1,    # timeout (s)
                quiet=True,
            )
            for m in self.context_collection('STATUSTEXT'):
                if m.text.startswith("Flare "):
                    flare = m
            self.delay_sim_time(0.2, reason="rate-limit SET_HAGL")
        # the flare message reports the height used to trigger it:
        if not flare.text.startswith("Flare 0.5m"):
            raise NotAchievedException(f"Flare was not triggered by the external HAGL: {flare.text}")
        alt = self.get_altitude(relative=True)
        if alt < 10:
            raise NotAchievedException(f"Flare at {alt}m could have come from the vehicle's own height estimate")

        self.wait_disarmed(timeout=120)
