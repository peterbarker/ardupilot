'''
Plane autotest: AcroLocking

AP_FLAKE8_CLEAN
'''

from test_discovery import TestGroup
from test_discovery import testcase


class AcroLocking(TestGroup):
    @testcase(bucket="PlaneTests1b")
    def AcroLocking(self):
        '''Plane acro-locking mode'''

        model = "plane-3d"

        self.customise_SITL_commandline(
            [],
            model=model,
            defaults_filepath="Tools/autotest/models/plane-3d.parm",
            wipe=True)
        self.set_parameters({
            'ACRO_LOCKING': 2,
        })

        self.takeoff(150, mode='TAKEOFF', timeout=120)
        self.change_mode('ACRO')

        self.set_rc(3, 2000)

        self.progress("Pitching")
        self.set_rc(2, 1700)
        self.wait_pitch(20, accuracy=5)
        self.set_rc(2, 1500)

        self.progress("Rolling")
        self.set_rc(1, 1550)
        self.wait_roll(90, accuracy=2)
        self.set_rc(1, 1500)
        self.wait_roll(90, accuracy=5, minimum_duration=20)
        # introduce some servo trim error to make sure acro mode does
        # its job
        self.context_push()
        self.set_parameters({
            "SERVO1_TRIM": 1550,
        })
        self.wait_roll(90, accuracy=5, minimum_duration=20, timeout=60)
        self.context_pop()

        self.set_rc(3, 1000)
        self.disarm_vehicle(force=True)
