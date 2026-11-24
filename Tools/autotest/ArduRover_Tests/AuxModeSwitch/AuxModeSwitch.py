'''
Rover autotest: AuxModeSwitch

AP_FLAKE8_CLEAN
'''

from test_discovery import TestGroup
from test_discovery import testcase


class AuxModeSwitch(TestGroup):
    @testcase(bucket="Rover")
    def AuxModeSwitch(self):
        '''Set modes via auxswitches'''
        # from mavproxy_rc.py
        mapping = [0, 1165, 1295, 1425, 1555, 1685, 1815]
        self.set_parameter("MODE1", 1)  # acro
        self.set_rc(8, mapping[1])
        self.wait_mode('ACRO')

        self.set_rc_from_map({9: 1000, 10: 1000})
        self.set_parameters({
            "RC9_OPTION": 53, # steering
            "RC10_OPTION": 54, # hold
        })
        self.set_rc(9, 1900)
        self.wait_mode("STEERING")
        self.set_rc(10, 1900)
        self.wait_mode("HOLD")

        # reset both switches - should go back to ACRO
        self.set_rc_from_map({9: 1000, 10: 1000})
        self.wait_mode("ACRO")

        self.set_rc(9, 1900)
        self.wait_mode("STEERING")
        self.set_rc(10, 1900)
        self.wait_mode("HOLD")

        self.set_rc(10, 1000) # this re-polls the mode switch
        self.wait_mode("ACRO")
        self.set_rc(9, 1000)
