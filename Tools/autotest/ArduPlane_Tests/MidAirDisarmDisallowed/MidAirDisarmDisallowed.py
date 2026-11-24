'''
Plane autotest: MidAirDisarmDisallowed

AP_FLAKE8_CLEAN
'''

from vehicle_test_suite import NotAchievedException

from test_discovery import TestGroup
from test_discovery import testcase


class MidAirDisarmDisallowed(TestGroup):
    @testcase(bucket="PlaneTests1a")
    def MidAirDisarmDisallowed(self):
        '''Ensure mid-air disarm is not possible'''
        self.takeoff(50)
        disarmed = False
        try:
            self.disarm_vehicle()
            disarmed = True
        except ValueError as e:
            self.progress("Got %s" % repr(e))
            if "Expected MAV_RESULT_ACCEPTED got MAV_RESULT_FAILED" not in str(e):
                raise e
        if disarmed:
            raise NotAchievedException("Disarmed when we shouldn't have")
        # should still be able to force-disarm:
        self.disarm_vehicle(force=True)
        self.reboot_sitl()
