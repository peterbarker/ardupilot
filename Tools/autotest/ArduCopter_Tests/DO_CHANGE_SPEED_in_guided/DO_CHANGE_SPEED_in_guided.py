'''
Copter autotest: DO_CHANGE_SPEED_in_guided

AP_FLAKE8_CLEAN
'''

from pymavlink import mavutil

from vehicle_test_suite import AltFrame

from test_discovery import TestGroup
from test_discovery import testcase


class DO_CHANGE_SPEED_in_guided(TestGroup):
    @testcase(bucket="CopterTests2c")
    def DO_CHANGE_SPEED_in_guided(self):
        '''test Copter DO_CHANGE_SPEED handling in guided mode'''
        self.takeoff(20, mode='GUIDED')

        new_loc = self.get_location()
        new_loc_offset_n = 2000
        new_loc_offset_e = 0
        self.location_offset_ne(new_loc, new_loc_offset_n, new_loc_offset_e)

        second_loc_offset_n = -1000
        second_loc_offset_e = 0
        second_loc = self.get_location()
        self.location_offset_ne(second_loc, second_loc_offset_n, second_loc_offset_e)

        # for run_cmd we fly away from home
        for (tloc, command) in (new_loc, self.run_cmd), (second_loc, self.run_cmd_int):
            self.run_cmd_int(
                mavutil.mavlink.MAV_CMD_DO_REPOSITION,
                p1=-1,  # "default"
                p2=0,   # flags; none supplied here
                p3=0,   # loiter radius for planes, zero ignored
                p4=float("nan"),  # nan means do whatever you want to do
                p5=int(tloc.lat * 1e7),
                p6=int(tloc.lng * 1e7),
                p7=tloc.get_alt_m(AltFrame.ABSOLUTE),
                frame=mavutil.mavlink.MAV_FRAME_GLOBAL,
            )
            for speed in [2, 10, 4]:
                command(
                    mavutil.mavlink.MAV_CMD_DO_CHANGE_SPEED,
                    p1=1,  # groundspeed,
                    p2=speed,
                    p3=-1,  # throttle, -1 is no-change
                    p4=0,   # absolute value, not relative
                )
                self.wait_groundspeed(speed-0.2, speed+0.2, minimum_duration=10, timeout=20)

        # we've made random changes to vehicle guided speeds above;
        # reboot vehicle to reset those:
        self.disarm_vehicle(force=True)
        self.reboot_sitl()
