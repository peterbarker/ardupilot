'''
Plane autotest: SDCardWPTest

AP_FLAKE8_CLEAN
'''

from test_discovery import TestGroup
from test_discovery import testcase


class SDCardWPTest(TestGroup):
    @testcase(bucket="PlaneTests1a")
    def SDCardWPTest(self):
        '''test BRD_SD_MISSION support'''
        spiral_script = "mission_spiral.lua"

        self.context_push()
        self.install_example_script_context(spiral_script)
        self.context_collect('STATUSTEXT')
        self.set_parameters({
            "BRD_SD_MISSION" : 64,
            "SCR_ENABLE" : 1,
            "SCR_VM_I_COUNT" : 1000000
            })

        self.wait_ready_to_arm()
        self.reboot_sitl()

        self.wait_text("Loaded spiral mission creator", check_context=True)
        self.set_parameters({
            "SCR_USER2": 19, # radius
            "SCR_USER3": -35.36322, # lat
            "SCR_USER4": 149.16525, # lon
            "SCR_USER5": 684.13, # alt
        })

        count = (65536 // 15) - 1

        self.progress("Creating spiral mission of size %s" % count)
        self.set_parameter("SCR_USER1", count)

        self.wait_text("Created spiral of size %u" % count, check_context=True)

        self.progress("Checking spiral before reboot")
        self.set_parameter("SCR_USER6", count)
        self.wait_text("Compared spiral of size %u OK" % count, check_context=True)
        self.set_parameter("SCR_USER6", 0)

        self.wait_ready_to_arm()
        self.reboot_sitl()
        self.progress("Checking spiral after reboot")
        self.set_parameter("SCR_USER6", count)
        self.wait_text("Compared spiral of size %u OK" % count, check_context=True)

        self.context_pop()
        self.wait_ready_to_arm()
        self.reboot_sitl()
