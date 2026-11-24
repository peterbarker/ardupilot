'''
Copter autotest: InvalidJumpTags

AP_FLAKE8_CLEAN
'''

from pymavlink import mavutil

from test_discovery import TestGroup
from test_discovery import testcase


class InvalidJumpTags(TestGroup):
    @testcase(bucket="CopterTests2c")
    def InvalidJumpTags(self):
        '''Verify the behaviour when selecting invalid jump tags.'''

        MAX_TAG_NUM = 65535
        # Jump tag is not present, so expect FAILED
        self.run_cmd(mavutil.mavlink.MAV_CMD_DO_JUMP_TAG,
                     p1=MAX_TAG_NUM,
                     timeout=1,
                     want_result=mavutil.mavlink.MAV_RESULT_FAILED)

        # Jump tag is too big, so expect DENIED
        self.run_cmd(mavutil.mavlink.MAV_CMD_DO_JUMP_TAG,
                     p1=MAX_TAG_NUM+1,
                     timeout=1,
                     want_result=mavutil.mavlink.MAV_RESULT_DENIED)
