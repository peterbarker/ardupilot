'''
Copter autotest: Clamp

AP_FLAKE8_CLEAN
'''

from pymavlink import mavutil

from test_discovery import TestGroup
from test_discovery import testcase


class Clamp(TestGroup):
    @testcase(bucket="CopterTests1e")
    def Clamp(self):
        '''test Copter docking clamp'''
        # the simulated clamp only grabs within 0.5m of home
        # (SIM_Aircraft.cpp), but the suite lets a test leave the vehicle
        # up to max_distance_from_startup_location_at_end_of_test() away,
        # which is 2m for Copter - four times the grab radius.  Restart
        # the simulator so we begin where the clamp expects us: a failing
        # run inherited an 0.59m offset and got "Clamp: missed vehicle".
        self.reset_SITL_commandline()

        clamp_ch = 11
        self.set_parameters({
            "SIM_CLAMP_CH": clamp_ch,
        })

        self.takeoff(1, mode='LOITER')

        self.context_push()
        self.context_collect('STATUSTEXT')
        self.progress("Ensure can't take off with clamp in place")
        self.run_cmd(mavutil.mavlink.MAV_CMD_DO_SET_SERVO, p1=11, p2=2000)
        self.wait_statustext("SITL: Clamp: grabbed vehicle", check_context=True)
        self.arm_vehicle()
        self.set_rc(3, 2000)
        self.wait_altitude(0, 5, minimum_duration=5, relative=True)
        self.run_cmd(mavutil.mavlink.MAV_CMD_DO_SET_SERVO, p1=11, p2=1000)
        self.wait_statustext("SITL: Clamp: released vehicle", check_context=True)
        self.wait_altitude(5, 5000, minimum_duration=1, relative=True)
        self.do_RTL()
        self.set_rc(3, 1000)
        self.change_mode('LOITER')
        self.context_pop()

        self.progress("Same again for repeatability")
        self.context_push()
        self.context_collect('STATUSTEXT')
        self.run_cmd(mavutil.mavlink.MAV_CMD_DO_SET_SERVO, p1=11, p2=2000)
        self.wait_statustext("SITL: Clamp: grabbed vehicle", check_context=True)
        self.arm_vehicle()
        self.set_rc(3, 2000)
        self.wait_altitude(0, 1, minimum_duration=5, relative=True)
        self.run_cmd(mavutil.mavlink.MAV_CMD_DO_SET_SERVO, p1=11, p2=1000)
        self.wait_statustext("SITL: Clamp: released vehicle", check_context=True)
        self.wait_altitude(5, 5000, minimum_duration=1, relative=True)
        self.do_RTL()
        self.set_rc(3, 1000)
        self.change_mode('LOITER')
        self.context_pop()

        here = self.get_location()
        loc = self.offset_location_ne(here, 10, 0)
        self.takeoff(5, mode='GUIDED')
        self.send_do_reposition(loc)
        self.wait_location(loc, timeout=120)
        self.land_and_disarm()

        # explicitly set home so we RTL to the right spot
        self.set_home(here)

        self.context_push()
        self.context_collect('STATUSTEXT')
        self.run_cmd(mavutil.mavlink.MAV_CMD_DO_SET_SERVO, p1=11, p2=2000)
        self.wait_statustext("SITL: Clamp: missed vehicle", check_context=True)
        self.run_cmd(mavutil.mavlink.MAV_CMD_DO_SET_SERVO, p1=11, p2=1000)
        self.context_pop()

        self.takeoff(5, mode='GUIDED')
        self.do_RTL()

        self.takeoff(5, mode='GUIDED')
        self.land_and_disarm()

        self.context_push()
        self.context_collect('STATUSTEXT')
        self.run_cmd(mavutil.mavlink.MAV_CMD_DO_SET_SERVO, p1=11, p2=2000)
        self.wait_statustext("SITL: Clamp: grabbed vehicle", check_context=True)
        self.context_pop()
