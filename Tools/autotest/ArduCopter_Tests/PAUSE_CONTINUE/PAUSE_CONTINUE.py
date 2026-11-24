'''
Copter autotest: PAUSE_CONTINUE

AP_FLAKE8_CLEAN
'''

from test_discovery import TestGroup
from test_discovery import testcase


class PAUSE_CONTINUE(TestGroup):
    @testcase(bucket="CopterTests2b")
    def PAUSE_CONTINUE(self):
        '''Test MAV_CMD_DO_PAUSE_CONTINUE in AUTO mode'''
        self.load_mission(filename="copter_mission.txt", strict=False)
        self.set_parameter(name="AUTO_OPTIONS", value=3)
        self.change_mode(mode="AUTO")
        self.wait_ready_to_arm()
        self.arm_vehicle()

        self.wait_current_waypoint(wpnum=3, timeout=500)
        self.send_pause_command()
        self.wait_groundspeed(speed_min=0, speed_max=1, minimum_duration=5)
        self.send_resume_command()

        self.wait_current_waypoint(wpnum=4, timeout=500)
        self.send_pause_command()
        self.wait_groundspeed(speed_min=0, speed_max=1, minimum_duration=5)
        self.send_resume_command()

        # sending a pause, or resume, to the aircraft twice, doesn't result in reporting a failure
        self.wait_current_waypoint(wpnum=5, timeout=500)
        self.send_pause_command()
        self.send_pause_command()
        self.wait_groundspeed(speed_min=0, speed_max=1, minimum_duration=5)
        self.send_resume_command()
        self.send_resume_command()

        self.wait_disarmed(timeout=500)
