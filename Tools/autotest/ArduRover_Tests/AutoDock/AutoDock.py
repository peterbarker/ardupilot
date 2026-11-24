'''
Rover autotest: AutoDock

AP_FLAKE8_CLEAN
'''

from vehicle_test_suite import NotAchievedException

from test_discovery import TestGroup
from test_discovery import testcase


class AutoDock(TestGroup):
    @testcase(bucket="Rover")
    def AutoDock(self):
        '''Test automatic docking of rover for multiple FOVs of simulated beacon'''
        self.set_parameters({
            "PLND_ENABLED": 1,
            "PLND_TYPE": 4,
            "PLND_ORIENT": 0,
        })

        # the loop below reboots, which returns the vehicle to the SITL
        # startup location; take our reference from there rather than
        # from wherever a previous test left us, or the target ends up
        # that much further away than the drive to it allows for:
        self.reboot_sitl()
        start = self.get_location()
        target = self.offset_location_ne(start, 50, 0)
        self.progress("Setting target to %f %f" % (start.lat, start.lng))
        stopping_dist = 0.5

        self.set_parameters({
            "SIM_PLD_ENABLE": 1,
            "SIM_PLD_LAT": target.lat,
            "SIM_PLD_LON": target.lng,
            "SIM_PLD_HEIGHT": 0,
            "SIM_PLD_ALT_LMT": 30,
            "SIM_PLD_DIST_LMT": 30,
            "SIM_PLD_ORIENT": 4,    # emit beams towards south, vehicle's heading must be north to see it
            "SIM_PLD_OPTIONS": 1,
            "DOCK_SPEED": 2,
            "DOCK_STOP_DIST": stopping_dist,
        })

        for type in range(0, 3):  # CYLINDRICAL FOV, CONICAL FOV, SPHERICAL FOV
            self.set_parameter("SIM_PLD_TYPE", type)
            self.reboot_sitl()
            self.change_mode('GUIDED')
            self.wait_ready_to_arm()
            self.arm_vehicle()
            initial_position = self.offset_location_ne(target, -20, -2)
            self.drive_to_location(initial_position)
            self.change_mode(8) # DOCK mode
            max_delta = 1
            self.wait_distance_to_location(target, 0, max_delta, timeout=180)
            self.disarm_vehicle()
            self.assert_receive_message('GLOBAL_POSITION_INT')
            new_pos = self.get_location()
            delta = abs(self.get_distance(target, new_pos) - stopping_dist)
            self.progress("Docked %f metres from stopping point" % delta)
            if delta > max_delta:
                raise NotAchievedException("Did not dock close enough to stopping point (%fm > %fm" % (delta, max_delta))

            if not self.current_onboard_log_contains_message("PL"):
                raise NotAchievedException("Did not see expected PL message")

        self.progress("All done")
