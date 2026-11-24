'''
Tracker autotest: Tracking

AP_FLAKE8_CLEAN
'''

import math

from vehicle_test_suite import AltFrame
from vehicle_test_suite import Location

from test_discovery import TestGroup
from test_discovery import testcase


class Tracking(TestGroup):
    @testcase(bucket="Tracker")
    def Tracking(self):
        '''check the tracker points at a vehicle sending its position'''
        self.change_mode("AUTO")
        self.wait_ready_to_arm()

        m = self.assert_receive_message('GLOBAL_POSITION_INT')
        tracker_loc = Location(m.lat * 1.0e-7, m.lon * 1.0e-7, m.alt * 1.0e-3, AltFrame.ABSOLUTE)
        # the vehicle reports the tracker's own pressure, so the
        # tracker sees no altitude difference and should pitch to zero
        m = self.assert_receive_message('SCALED_PRESSURE')
        vehicle_press_abs = m.press_abs
        vehicle_temperature = m.temperature

        vehicle_loc = None
        last_sent = 0

        def send_vehicle_position(mav, m):
            nonlocal last_sent
            now = self.get_sim_time_cached()
            if vehicle_loc is None or now - last_sent < 0.1:
                return
            last_sent = now
            self.mav.mav.global_position_int_send(
                int(now * 1000),  # time_boot_ms
                int(vehicle_loc.lat * 1e7),
                int(vehicle_loc.lng * 1e7),
                int(vehicle_loc.get_alt_m(AltFrame.ABSOLUTE) * 1000),
                0,  # relative_alt
                0,  # vx
                0,  # vy
                0,  # vz
                65535  # hdg
            )
            self.mav.mav.scaled_pressure_send(
                int(now * 1000),  # time_boot_ms
                vehicle_press_abs,
                0,  # press_diff
                vehicle_temperature
            )

        self.context_push()
        self.install_message_hook_context(send_vehicle_position)

        # the simulated tracker's yaw range is centred on west and
        # spans 170 degrees either way, so keep the vehicle to the west
        for (north, east) in (0, -20), (20, -20), (-20, -20), (30, -40), (-30, -40):
            desyaw = math.degrees(math.atan2(east, north))
            self.start_subtest(f"Vehicle {north}m north {east}m east (bearing {desyaw:.0f})")
            vehicle_loc = self.offset_location_ne(tracker_loc, north, east)
            self.wait_attitude(despitch=0, desyaw=desyaw, tolerance=2, timeout=120, minimum_duration=5)

        self.context_pop()
