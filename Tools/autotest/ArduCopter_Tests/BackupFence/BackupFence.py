'''
Copter autotest: BackupFence

AP_FLAKE8_CLEAN
'''

from pymavlink import mavutil

from vehicle_test_suite import NotAchievedException

from test_discovery import TestGroup
from test_discovery import testcase


class BackupFence(TestGroup):
    def assert_fence_not_breached(self):
        m = self.assert_receive_message('FENCE_STATUS', timeout=2)
        if m.breach_status != 0:
            raise NotAchievedException("Fence is breached")

    def assert_fence_breached(self):
        m = self.assert_receive_message('FENCE_STATUS', timeout=2)
        if m.breach_status == 0:
            raise NotAchievedException("Fence is not breached")

    @testcase(bucket="CopterTests2d")
    def BackupFence(self):
        '''ensure the lateral backup fence functionality works'''
        self.set_parameters({
            "FENCE_RADIUS": 10,
            "FENCE_TYPE": 2,  # circle only
            "FENCE_ENABLE": 1,
            "AVOID_ENABLE": 0,
        })
        self.wait_sensor_state(mavutil.mavlink.MAV_SYS_STATUS_GEOFENCE,
                               present=True,
                               enabled=True,
                               healthy=True,
                               timeout=30)
        self.assert_fence_not_breached()
        self.takeoff(10, mode='LOITER')
        self.set_rc(2, 1300)
        self.wait_mode('RTL', timeout=30)
        self.assert_fence_breached()
        self.change_mode('LOITER')
        # continue to suck as a pilot
        self.wait_mode('RTL', timeout=30)
        self.assert_fence_breached()
        self.set_rc(2, 1500)  # stop sucking
        self.wait_disarmed()
        self.assert_at_home()
