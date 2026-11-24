'''
Copter autotest: RecordThenPlayMission

AP_FLAKE8_CLEAN
'''

import os

from vehicle_test_suite import NotAchievedException

from test_discovery import TestGroup
from test_discovery import testcase

from arducopter import testdir


class RecordThenPlayMission(TestGroup):
    @testcase(bucket="CopterTests2a")
    def RecordThenPlayMission(self, side=50, timeout=300):
        '''Use switches to toggle in mission, then fly it'''
        self.takeoff(20, mode="ALT_HOLD")

        """Fly a square, flying N then E ."""
        tstart = self.get_sim_time()

        # ensure all sticks in the middle
        self.set_rc_from_map({
            1: 1500,
            2: 1500,
            3: 1500,
            4: 1500,
        })

        # switch to loiter mode temporarily to stop us from rising
        self.change_mode('LOITER')

        # first aim north
        self.progress("turn right towards north")
        self.set_rc(4, 1580)
        self.wait_heading(10)
        self.set_rc(4, 1500)

        # save bottom left corner of box as waypoint
        self.progress("Save WP 1 & 2")
        self.save_wp()

        # switch back to ALT_HOLD mode
        self.change_mode('ALT_HOLD')

        # pitch forward to fly north
        self.progress("Going north %u meters" % side)
        self.set_rc(2, 1300)
        self.wait_distance(side)
        self.set_rc(2, 1500)

        # save top left corner of square as waypoint
        self.progress("Save WP 3")
        self.save_wp()

        # roll right to fly east
        self.progress("Going east %u meters" % side)
        self.set_rc(1, 1700)
        self.wait_distance(side)
        self.set_rc(1, 1500)

        # save top right corner of square as waypoint
        self.progress("Save WP 4")
        self.save_wp()

        # pitch back to fly south
        self.progress("Going south %u meters" % side)
        self.set_rc(2, 1700)
        self.wait_distance(side)
        self.set_rc(2, 1500)

        # save bottom right corner of square as waypoint
        self.progress("Save WP 5")
        self.save_wp()

        # roll left to fly west
        self.progress("Going west %u meters" % side)
        self.set_rc(1, 1300)
        self.wait_distance(side)
        self.set_rc(1, 1500)

        # save bottom left corner of square (should be near home) as waypoint
        self.progress("Save WP 6")
        self.save_wp()

        # reduce throttle again
        self.set_rc(3, 1500)

        # descend to 10m
        self.progress("Descend to 10m in Loiter")
        self.change_mode('LOITER')
        self.set_rc(3, 1200)
        time_left = timeout - (self.get_sim_time() - tstart)
        self.progress("timeleft = %u" % time_left)
        if time_left < 20:
            time_left = 20
        self.wait_altitude(-10, 10, timeout=time_left, relative=True)
        self.set_rc(3, 1500)
        self.save_wp()

        # Disarm and reset before testing mission
        self.land_and_disarm()
        self.set_rc(3, 1000)
        self.change_mode('LOITER')

        # save the stored mission to file
        mavproxy = self.start_mavproxy()
        num_wp = self.save_mission_to_file_using_mavproxy(
            mavproxy,
            os.path.join(testdir, "ch7_mission.txt"))
        self.stop_mavproxy(mavproxy)
        if not num_wp:
            raise NotAchievedException("save_mission_to_file failed")

        self.arm_vehicle()
        self.progress("test: Fly a mission from 1 to %u" % num_wp)
        self.change_mode('AUTO')
        # Raise throttle in auto to trigger takeoff
        self.set_rc(3, 1500)
        self.wait_waypoint(0, num_wp-1, timeout=500)
        self.progress("test: MISSION COMPLETE: passed!")
        self.land_and_disarm()

        # we are not at the home location - reboot so the next test starts there
        self.reboot_sitl()
