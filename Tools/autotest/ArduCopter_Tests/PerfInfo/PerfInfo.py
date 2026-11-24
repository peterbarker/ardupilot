'''
Copter autotest: PerfInfo

AP_FLAKE8_CLEAN
'''

from vehicle_test_suite import NotAchievedException

from test_discovery import TestGroup
from test_discovery import testcase


class PerfInfo(TestGroup):
    @testcase(bucket="CopterTests1d")
    def PerfInfo(self):
        '''Test Scheduler PerfInfo output'''
        self.set_parameter('SCHED_OPTIONS', 1)  # enable gathering
        # sometimes we need to trigger collection....
        content = self.fetch_file_via_ftp("@SYS/tasks.txt")
        self.delay_sim_time(5, reason="scheduler perf data to accumulate")
        content = self.fetch_file_via_ftp("@SYS/tasks.txt")
        self.progress("Got content (%s)" % str(content))

        lines = content.split("\n")

        if not lines[0].startswith("TasksV2"):
            raise NotAchievedException("Expected TasksV2 as first line first not (%s)" % lines[0])
        # last line is empty, so -2 here
        if not lines[-2].startswith("AP_Vehicle::update_arming"):
            raise NotAchievedException("Expected EFI last not (%s)" % lines[-2])
