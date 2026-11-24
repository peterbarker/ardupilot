'''
Copter autotest: ScriptCopterPosOffsets

AP_FLAKE8_CLEAN
'''

from pymavlink import mavutil

from test_discovery import TestGroup
from test_discovery import testcase


class ScriptCopterPosOffsets(TestGroup):
    @testcase(bucket="CopterTests1e")
    def ScriptCopterPosOffsets(self):
        '''test the copter-posoffset.lua example script'''
        self.context_push()

        # enable scripting and arming/takingoff in Auto mode
        self.set_parameters({
            "SCR_ENABLE": 1,
            "AUTO_OPTIONS": 3,
            "RC12_OPTION": 300
        })
        self.reboot_sitl()

        # install copter-posoffset script
        self.install_example_script_context('copter-posoffset.lua')
        self.reboot_sitl()

        # create simple mission with a single takeoff command
        self.upload_simple_relhome_mission([
            (mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 20)
        ])

        # switch to loiter to wait for position estimate (aka GPS lock)
        self.change_mode('LOITER')
        self.wait_ready_to_arm()

        # arm and takeoff in Auto mode
        self.change_mode('AUTO')
        self.arm_vehicle()

        # wait for vehicle to climb to at least 10m
        self.wait_altitude(8, 12, relative=True)

        # add position offset to East and confirm vehicle moves
        self.set_parameter("PSC_OFS_POS_E", 20)
        self.set_rc(12, 2000)
        self.wait_distance(18)

        # remove position offset and wait for vehicle to return home
        self.set_parameter("PSC_OFS_POS_E", 0)
        self.wait_distance_to_home(distance_min=0, distance_max=4, timeout=20)

        # add velocity offset and confirm vehicle moves
        self.set_parameter("PSC_OFS_VEL_N", 5)
        self.wait_groundspeed(4.8, 5.2, minimum_duration=5, timeout=20)

        # remove velocity offset and switch to RTL
        self.set_parameter("PSC_OFS_VEL_N", 0)
        self.set_rc(12, 1000)
        self.do_RTL()
        self.context_pop()
        self.reboot_sitl()
