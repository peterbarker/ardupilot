'''
Copter autotest: ThrowDoubleDrop

AP_FLAKE8_CLEAN
'''

from pymavlink import mavutil
from pymavlink.rotmat import Vector3

from test_discovery import TestGroup
from test_discovery import testcase


class ThrowDoubleDrop(TestGroup):
    @testcase(bucket="CopterTests1e")
    def ThrowDoubleDrop(self):
        '''Test a more complicated drop-mode scenario'''
        self.progress("Getting a lift to altitude")
        self.set_parameters({
            "SIM_SHOVE_Z": -11,
            "THROW_TYPE": 1,   # drop
            "MOT_SPOOL_TIME": 2,
        })
        self.change_mode('THROW')
        self.wait_ready_to_arm()
        self.arm_vehicle()
        try:
            self.set_parameter("SIM_SHOVE_TIME", 30000)
        except ValueError:
            # the shove resets this to zero
            pass

        self.wait_altitude(100, 1000, timeout=100, relative=True)
        self.context_collect('STATUSTEXT')
        self.wait_statustext("throw detected - spooling motors", check_context=True, timeout=10)
        self.wait_statustext("throttle is unlimited - uprighting", check_context=True)
        self.wait_statustext("uprighted - controlling height", check_context=True)
        self.wait_statustext("height achieved - controlling position", check_context=True)
        self.progress("Waiting for still")
        self.wait_speed_vector(Vector3(0, 0, 0))
        self.change_mode('ALT_HOLD')
        self.set_rc(3, 1000)
        self.wait_disarmed(timeout=90)
        self.zero_throttle()

        self.progress("second flight")
        self.upload_square_mission_items_around_location(self.poll_home_position())

        self.set_parameters({
            "THROW_NEXTMODE": 3,  # auto
        })

        self.change_mode('THROW')
        self.wait_ready_to_arm()
        self.arm_vehicle()
        try:
            self.set_parameter("SIM_SHOVE_TIME", 30000)
        except ValueError:
            # the shove resets this to zero
            pass

        self.wait_altitude(100, 1000, timeout=100, relative=True)
        self.wait_statustext("throw detected - spooling motors", check_context=True, timeout=10)
        self.wait_statustext("throttle is unlimited - uprighting", check_context=True)
        self.wait_statustext("uprighted - controlling height", check_context=True)
        self.wait_statustext("height achieved - controlling position", check_context=True)
        self.wait_mode('AUTO')
        self.wait_disarmed(timeout=240)

        # we are not at the home location - reboot so the next test starts there
        self.reboot_sitl()

    def upload_square_mission_items_around_location(self, loc):
        alt = 20
        loc.alt = alt
        items = [
            (mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, alt)
        ]

        for (ofs_n, ofs_e) in (20, 20), (20, -20), (-20, -20), (-20, 20), (20, 20):
            items.append((mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, ofs_n, ofs_e, alt))

        items.append((mavutil.mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH, 0, 0, 0))

        self.upload_simple_relhome_mission(items)
