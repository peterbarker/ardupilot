'''
Copter autotest: PayloadPlaceManualReleaseDelay

AP_FLAKE8_CLEAN
'''

from vehicle_test_suite import NotAchievedException

from test_discovery import TestGroup
from test_discovery import testcase


class PayloadPlaceManualReleaseDelay(TestGroup):
    @testcase(bucket="CopterTests2c")
    def PayloadPlaceManualReleaseDelay(self):
        '''check PLDP_DELAY is honoured after a manual release early in the descent'''
        self.set_analog_rangefinder_parameters()
        self.set_servo_gripper_parameters()
        self.set_parameters({
            "PLDP_DELAY": 5,
            # slow the descent ramp so the release happens before the descent speed is reached
            "PLDP_SPEED_DN": 1.5,
            "WP_ACC_Z": 0.5,
        })
        self.reboot_sitl()

        self.context_collect('STATUSTEXT')
        self.run_auxfunc(19, 2)
        self.wait_statustext("Gripper load grabbed", check_context=True)

        num_wp = self.load_and_start_mission("copter_payload_place.txt")
        self.wait_current_waypoint(3, timeout=120)
        self.wait_climbrate(-5, -0.2, minimum_duration=0.2, timeout=60)

        self.progress("Releasing the gripper early in the descent")
        self.run_auxfunc(19, 0)
        self.wait_statustext("PayloadPlace: Manual release", check_context=True)
        # the defect only shows before the descent reaches 95% of PLDP_SPEED_DN (1.5 m/s)
        climb = self.assert_receive_message('VFR_HUD').climb
        if climb < -1.2:
            raise NotAchievedException("Released too late in the descent (climb=%f)" % climb)
        release_alt = self.get_altitude(relative=True)
        tstart = self.get_sim_time()
        while self.get_sim_time_cached() - tstart < 4:
            alt = self.get_altitude(relative=True)
            if alt > release_alt + 0.5:
                raise NotAchievedException("Climbed before PLDP_DELAY expired (alt=%f release_alt=%f)" %
                                           (alt, release_alt))

        self.progress("Waiting for the ascent once PLDP_DELAY expires")
        self.wait_altitude(release_alt + 0.5, 22, relative=True, timeout=60)
        self.wait_waypoint(num_wp-1, num_wp-1)
        self.do_RTL()
