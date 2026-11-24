'''
Copter autotest: AltTypes

AP_FLAKE8_CLEAN
'''

from vehicle_test_suite import AltFrame
from vehicle_test_suite import Location
from vehicle_test_suite import NotAchievedException

from test_discovery import TestGroup
from test_discovery import testcase


class AltTypes(TestGroup):
    @testcase(bucket="CopterTests1a")
    def AltTypes(self):
        '''Test Different Altitude Types'''
        '''start by disabling GCS failsafe, otherwise we immediately disarm
        due to (apparently) not receiving traffic from the GCS for
        too long.  This is probably a function of --speedup'''

        '''this test flies the vehicle somewhere lower than were it started.
        It then disarms.  It then arms, which should reset home to the
        new, lower altitude.  This delta should be outside 1m but
        within a few metres of the old one.

        '''

        self.install_terrain_handlers_context()

        self.set_parameter("FS_GCS_ENABLE", 0)
        self.change_mode('GUIDED')
        self.wait_ready_to_arm()
        self.arm_vehicle()
        m = self.assert_receive_message('GLOBAL_POSITION_INT')
        max_initial_home_alt_m = 500
        if m.relative_alt > max_initial_home_alt_m:
            raise NotAchievedException("Initial home alt too high (%fm > %fm)" %
                                       (m.relative_alt*1000, max_initial_home_alt_m*1000))
        orig_home_offset_mm = m.alt - m.relative_alt
        self.user_takeoff(5)

        self.progress("Flying to low position")
        current_alt = self.get_location().get_alt_m(AltFrame.ABSOLUTE)
# 10m delta        low_position = Location(-35.358273, 149.169165, current_alt, AltFrame.ABSOLUTE)
        low_position = Location(-35.36200016, 149.16415599, current_alt, AltFrame.ABSOLUTE)
        self.fly_guided_move_to(low_position, timeout=240)
        self.change_mode('LAND')
        # expecting home to change when disarmed
        self.wait_landed_and_disarmed()
        # wait a while for home to move (it shouldn't):
        self.delay_sim_time(10, reason="confirming home does not move after landing")
        m = self.assert_receive_message('GLOBAL_POSITION_INT')
        new_home_offset_mm = m.alt - m.relative_alt
        home_offset_delta_mm = orig_home_offset_mm - new_home_offset_mm
        self.progress("new home offset: %f delta=%f" %
                      (new_home_offset_mm, home_offset_delta_mm))
        self.progress("gpi=%s" % str(m))
        max_home_offset_delta_mm = 10
        if home_offset_delta_mm > max_home_offset_delta_mm:
            raise NotAchievedException("Large home offset delta: want<%f got=%f" %
                                       (max_home_offset_delta_mm, home_offset_delta_mm))
        self.progress("Ensuring home moves when we arm")
        self.change_mode('GUIDED')
        self.wait_ready_to_arm()
        self.arm_vehicle()
        m = self.assert_receive_message('GLOBAL_POSITION_INT')
        post_arming_home_offset_mm = m.alt - m.relative_alt
        self.progress("post-arming home offset: %f" % (post_arming_home_offset_mm))
        self.progress("gpi=%s" % str(m))
        min_post_arming_home_offset_delta_mm = -2500
        max_post_arming_home_offset_delta_mm = -4000
        delta_between_original_home_alt_offset_and_new_home_alt_offset_mm = post_arming_home_offset_mm - orig_home_offset_mm
        self.progress("delta=%f-%f=%f" % (
            post_arming_home_offset_mm,
            orig_home_offset_mm,
            delta_between_original_home_alt_offset_and_new_home_alt_offset_mm))
        self.progress("Home moved %fm vertically" % (delta_between_original_home_alt_offset_and_new_home_alt_offset_mm/1000.0))
        if delta_between_original_home_alt_offset_and_new_home_alt_offset_mm > min_post_arming_home_offset_delta_mm:
            raise NotAchievedException(
                "Home did not move vertically on arming: want<=%f got=%f" %
                (min_post_arming_home_offset_delta_mm, delta_between_original_home_alt_offset_and_new_home_alt_offset_mm))
        if delta_between_original_home_alt_offset_and_new_home_alt_offset_mm < max_post_arming_home_offset_delta_mm:
            raise NotAchievedException(
                "Home moved too far vertically on arming: want>=%f got=%f" %
                (max_post_arming_home_offset_delta_mm, delta_between_original_home_alt_offset_and_new_home_alt_offset_mm))

        self.wait_disarmed()

        # we are not at the home location - reboot so the next test starts there
        self.reboot_sitl()
