'''
Plane autotest: AerobaticsScripting

AP_FLAKE8_CLEAN
'''

from vehicle_test_suite import NotAchievedException

from test_discovery import TestGroup
from test_discovery import testcase


class AerobaticsScripting(TestGroup):
    @testcase(bucket="PlaneTests1a")
    def AerobaticsScripting(self):
        '''Fixed Wing Aerobatics'''
        applet_script = "Aerobatics/FixedWing/plane_aerobatics.lua"
        airshow = "Aerobatics/FixedWing/Schedules/AirShow.txt"
        trick72 = "trick72.txt"

        model = "plane-3d"

        self.customise_SITL_commandline(
            [],
            model=model,
            wipe=True)

        self.context_push()
        self.install_applet_script_context(applet_script)
        self.install_applet_script_context(airshow, install_name=trick72)
        self.context_collect('STATUSTEXT')
        self.reboot_sitl()

        self.set_parameter("TRIK_ENABLE", 1)
        self.set_rc(7, 1000) # disable tricks

        self.scripting_restart()
        self.wait_text("Enabled 3 aerobatic tricks", check_context=True)
        self.set_parameters({
            "TRIK1_ID": 72,
            "RC7_OPTION" : 300, # activation switch
            "RC9_OPTION" : 301, # selection switch
            "SIM_SPEEDUP": 5, # need to give some cycles to lua
        })

        self.wait_ready_to_arm()
        self.change_mode("TAKEOFF")
        self.arm_vehicle()
        self.wait_altitude(30, 40, timeout=30, relative=True)
        self.change_mode("CRUISE")

        self.set_rc(9, 1000) # select first trick
        self.delay_sim_time(1, reason="trick selection to register")
        self.set_rc(7, 1500) # show selected trick

        self.wait_text("Trick 1 selected (SuperAirShow)", check_context=True)
        self.set_rc(7, 2000) # activate trick
        self.wait_text("Trick 1 started (SuperAirShow)", check_context=True)

        highest_error = 0
        while True:
            m = self.mav.recv_match(type='NAMED_VALUE_FLOAT', timeout=2, blocking=True)
            if not m:
                break
            if m.name != 'PERR':
                continue
            highest_error = max(highest_error, m.value)
            if highest_error > 15:
                raise NotAchievedException("path error %.1f" % highest_error)

        if highest_error == 0:
            raise NotAchievedException("path error not reported")
        self.progress("Finished trick, max error=%.1fm" % highest_error)
        self.disarm_vehicle(force=True)

        messages = self.context_collection('STATUSTEXT')
        self.context_pop()
        self.reboot_sitl()

        # check all messages to see if we got all tricks
        tricks = ["Loop", "HalfReverseCubanEight", "ScaleFigureEight", "Immelmann",
                  "Split-S", "RollingCircle", "HumptyBump", "HalfCubanEight",
                  "BarrelRoll", "CrossBoxTopHat", "TriangularLoop",
                  "Finishing SuperAirShow!"]
        texts = [m.text for m in messages]
        for t in tricks:
            if t in texts:
                self.progress("Completed trick %s" % t)
            else:
                raise NotAchievedException("Missing trick %s" % t)
