'''
Plane autotest: AutoLandMode

AP_FLAKE8_CLEAN
'''

from vehicle_test_suite import Location
from vehicle_test_suite import NotAchievedException

from test_discovery import TestGroup
from test_discovery import testcase


class AutoLandMode(TestGroup):
    @testcase(bucket="PlaneTests1c")
    def AutoLandMode(self):
        '''Test AUTOLAND mode'''
        self.set_parameters({
            "AUTOLAND_DIR_OFF": 45,
            "TERRAIN_FOLLOW": 1,
            "AUTOLAND_CLIMB": 300,
        })
        self.customise_SITL_commandline(["--home", "-35.362938,149.165085,585,173"])
        self.context_collect('STATUSTEXT')
        self.load_mission("autoland_mission.txt")
        self.install_terrain_handlers_context()
        self.change_mode("AUTO")
        self.wait_ready_to_arm()
        self.arm_vehicle()
        self.wait_text("Autoland direction", check_context=True)
        self.wait_waypoint(2, 2, max_dist_to_final_wp_m=100)
        self.change_mode("AUTOLAND")
        self.wait_disarmed(400)
        self.progress("Check TECS target at start of landing approach")
        dfreader = self.dfreader_for_current_onboard_log()
        last_tecs_target = None
        checking_tecs_target = False
        saw_tecs_target = False
        checked_tecs_target = False
        while True:
            m = dfreader.recv_match(type=['MSG', 'TECS'])
            if m is None:
                break
            if m.get_type() == 'MSG':
                if m.Message.startswith("Landing approach start"):
                    checking_tecs_target = True
                    saw_tecs_target = False
                elif m.Message.startswith("Landing glide slope"):
                    checked_tecs_target = checking_tecs_target and saw_tecs_target
                    checking_tecs_target = False
            else:
                if checking_tecs_target:
                    saw_tecs_target = True
                    if last_tecs_target is None:
                        raise NotAchievedException("No TECS target before landing approach start")
                    if abs(m.hin - last_tecs_target) > 5:
                        raise NotAchievedException(
                            "TECS height target changed at landing approach start: %f to %f" %
                            (last_tecs_target, m.hin))
                last_tecs_target = m.hin
        if not checked_tecs_target:
            raise NotAchievedException("Did not check TECS target before landing glide slope")
        self.progress("Check the landed heading matches takeoff plus offset")
        self.wait_heading(218, accuracy=5, timeout=1)
        loc = Location.latlon_only(-35.362938, 149.165085)
        if self.get_distance(loc, self.get_location()) > 35:
            raise NotAchievedException("Did not land close to home")
        # bit 0 of AUTOLAND_OPTIONS is AUTOLAND_DIR_ON_ARM, which takes the
        # direction from the compass at arming rather than from ground course
        # later on.  TKOFF_OPTIONS bit 1, set here before, is not a defined
        # bit at all.
        self.set_parameters({
            "AUTOLAND_OPTIONS": 1,
        })
        self.wait_ready_to_arm()
        self.set_autodisarm_delay(0)
        # the first arm put an "Autoland direction" into the collection, and
        # matching that one would say nothing about this arm
        self.context_clear_collection('STATUSTEXT')
        self.arm_vehicle()
        self.progress("Check the set dir on arm option")
        self.wait_text("Autoland direction", check_context=True)
        # disarm rather than leaving it to the auto-disarm which the
        # context-pop restore happens to re-enable
        self.disarm_vehicle()
