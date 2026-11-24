'''
Rover autotest: GetMessageInterval

AP_FLAKE8_CLEAN
'''

from pymavlink import mavutil

from vehicle_test_suite import NotAchievedException

from test_discovery import TestGroup
from test_discovery import testcase


class GetMessageInterval(TestGroup):
    @testcase(bucket="Rover")
    def GetMessageInterval(self):
        '''check that the two methods for requesting a MESSAGE_INTERVAL message are equivalent'''
        target_msg = mavutil.mavlink.MAVLINK_MSG_ID_HOME_POSITION
        old_cmd = (mavutil.mavlink.MAV_CMD_GET_MESSAGE_INTERVAL, target_msg, 0)
        new_cmd = (mavutil.mavlink.MAV_CMD_REQUEST_MESSAGE, mavutil.mavlink.MAVLINK_MSG_ID_MESSAGE_INTERVAL, target_msg)

        interval_us = None

        for run_command in self.run_cmd, self.run_cmd_int:
            for cmd in old_cmd, new_cmd:
                cmd_id, p1, p2 = cmd

                self.context_collect("MESSAGE_INTERVAL")
                run_command(cmd_id, p1=p1, p2=p2)
                m = self.assert_receive_message('MESSAGE_INTERVAL', timeout=1, check_context=True)
                self.context_clear_collection("MESSAGE_INTERVAL")

                if m.message_id != target_msg:
                    raise NotAchievedException(f"Unexpected ID in MESSAGE_INTERVAL (want={target_msg}, got={m.message_id})")

                if interval_us is None:
                    interval_us = m.interval_us

                if m.interval_us != interval_us:
                    raise NotAchievedException(f"Unexpected interval_us (want={interval_us}, got={m.interval_us})")
