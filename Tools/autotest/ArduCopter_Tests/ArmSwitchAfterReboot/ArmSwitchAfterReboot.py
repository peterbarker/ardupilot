'''
Copter autotest: ArmSwitchAfterReboot

AP_FLAKE8_CLEAN
'''

from vehicle_test_suite import NotAchievedException

from test_discovery import TestGroup
from test_discovery import testcase


class ArmSwitchAfterReboot(TestGroup):
    @testcase(bucket="CopterTests1e")
    def ArmSwitchAfterReboot(self):
        '''test that the arming switch does not trigger after a reboot'''
        self.wait_ready_to_arm()
        self.set_parameters({
            "RC8_OPTION": 153,
        })
        self.set_rc(8, 2000)
        self.wait_armed()
        self.disarm_vehicle()
        self.context_collect('STATUSTEXT')
        self.reboot_sitl()

        tstart = self.get_sim_time()
        while True:
            if self.get_sim_time_cached() - tstart > 60:
                break
            if self.armed(poll=False):
                raise NotAchievedException("Armed after reboot with switch high")
            armmsg = self.statustext_in_collections('Arm: ')
            if armmsg is not None:
                raise NotAchievedException("statustext(%s) means we tried to arm" % armmsg.text)
        self.progress("Did not arm via arming switfch after a reboot")
