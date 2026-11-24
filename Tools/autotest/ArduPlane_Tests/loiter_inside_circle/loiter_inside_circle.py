'''
Plane autotest: loiter_inside_circle

AP_FLAKE8_CLEAN
'''

import vehicle_test_suite

from vehicle_test_suite import NotAchievedException

from test_discovery import TestGroup
from test_discovery import testcase


class loiter_inside_circle(TestGroup):
    @testcase(bucket="PlaneTests1a")
    def loiter_inside_circle(self):
        ''' Test a edge case when loiter is started inside the circle '''

        # Large radius triggers this issue
        self.set_parameter("WP_LOITER_RAD", 300)

        # Take off into straight and level FBWB flight
        self.takeoff_in_TAKEOFF(timeout=60)
        self.set_rc(3, 1500)

        # Change to FBWB at a consistent heading
        self.wait_heading(180, timeout=60)
        self.change_mode("FBWB")

        # A small amount of roll in the direction of the loiter makes the problem much worse
        # Stick mixing must be enabled so this roll is carried over into loiter
        self.set_rc(1, 1600)

        self.delay_sim_time(10, "wait a short time for vehicle to settle")

        # Push context so we can remove the message hook
        self.context_push()

        # Try and detect steps in roll output
        class DetectRollSteps(vehicle_test_suite.TestSuite.MessageHook):
            '''Checks for steps in roll demand'''

            def __init__(self, suite):
                super(DetectRollSteps, self).__init__(suite)
                self.nav_roll = None
                self.max_step = 0
                self.min_step = 0

            def hook_removed(self):
                if self.nav_roll is None:
                    raise NotAchievedException("Did not get NAV_CONTROLLER_OUTPUT")

            def process(self, mav, m):
                if m.get_type() != 'NAV_CONTROLLER_OUTPUT':
                    return

                new_nav_roll = m.nav_roll
                if self.nav_roll is None:
                    self.nav_roll = new_nav_roll
                    return

                roll_diff = new_nav_roll - self.nav_roll
                self.nav_roll = new_nav_roll

                self.max_step = max(self.max_step, roll_diff)
                self.min_step = min(self.min_step, roll_diff)

                if self.max_step > 60 and self.min_step < -60:
                    raise Exception("Large Roll step (%0.2f, %0.2f)" % (self.max_step, self.min_step))

        # Install a hook to check for the steppy roll output
        self.install_message_hook_context(DetectRollSteps(self))

        # Fly one loiter circle
        self.fly_LOITER(num_circles=1, timeout=120)

        # Pop context to remove message hook
        self.context_pop()

        # Done
        self.fly_home_land_and_disarm()
