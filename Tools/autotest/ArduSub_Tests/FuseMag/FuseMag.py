'''
Sub autotest: FuseMag

AP_FLAKE8_CLEAN
'''

from vehicle_test_suite import NotAchievedException

from test_discovery import TestGroup
from test_discovery import testcase

from ardusub import MagFuseSel
from ardusub import Joystick
from ardusub import MagCal


class FuseMag(TestGroup):
    def assert_mag_fusion_selection(self, expect_sel):
        """Get the most recent XKFS message and check the MAG_FUSION value"""
        self.progress(f"Expect mag fusion selection {expect_sel}")
        mlog = self.dfreader_for_current_onboard_log()
        found_sel = MagFuseSel.NOT_FUSING
        while True:
            m = mlog.recv_match(type='XKFS')
            if m is None:
                break
            found_sel = m.MAG_FUSION
        if found_sel != expect_sel:
            raise NotAchievedException(f"Expected mag fusion selection {expect_sel}, found {found_sel}")

    @testcase(bucket="Sub")
    def FuseMag(self):
        """Test EK3_MAG_CAL values"""

        # WHEN_FLYING: switch to FUSE_MAG after sub is armed for 5 seconds; switch to FUSE_YAW on disarm
        self.set_parameters({'EK3_MAG_CAL': MagCal.WHEN_FLYING})
        self.reboot_sitl()
        self.wait_ready_to_arm()
        self.assert_mag_fusion_selection(MagFuseSel.FUSE_YAW)
        self.arm_vehicle()
        self.delay_sim_time(10, reason="mag fusion to switch to FUSE_MAG")
        self.assert_mag_fusion_selection(MagFuseSel.FUSE_MAG)
        self.disarm_vehicle()
        self.delay_sim_time(1, reason="mag fusion to revert to FUSE_YAW")
        self.assert_mag_fusion_selection(MagFuseSel.FUSE_YAW)

        # AFTER_FIRST_CLIMB: switch to FUSE_MAG after sub is armed and descends 0.5m; switch to FUSE_YAW on disarm
        self.set_parameters({'EK3_MAG_CAL': MagCal.AFTER_FIRST_CLIMB})
        self.reboot_sitl()
        self.wait_ready_to_arm()
        self.assert_mag_fusion_selection(MagFuseSel.FUSE_YAW)
        altitude = self.get_altitude(relative=True)
        self.arm_vehicle()
        self.set_rc(Joystick.Throttle, 1300)
        self.wait_altitude(altitude_min=altitude-4, altitude_max=altitude-3, relative=False, timeout=60)
        self.set_rc(Joystick.Throttle, 1500)
        self.assert_mag_fusion_selection(MagFuseSel.FUSE_MAG)
        self.disarm_vehicle()
        self.delay_sim_time(1, reason="mag fusion to revert to FUSE_YAW")
        self.assert_mag_fusion_selection(MagFuseSel.FUSE_YAW)

        # ALWAYS
        self.set_parameters({'EK3_MAG_CAL': MagCal.ALWAYS})
        self.reboot_sitl()
        self.wait_ready_to_arm()
        self.assert_mag_fusion_selection(MagFuseSel.FUSE_MAG)

        # GROUND_AND_INFLIGHT: anchor yaw to the compass while disarmed and stationary, holding the
        # heading while the field states are learned; the anchor releases on arm and returns on disarm
        self.set_parameters({'EK3_MAG_CAL': MagCal.GROUND_AND_INFLIGHT})
        self.reboot_sitl()
        self.wait_ready_to_arm()
        self.assert_mag_fusion_selection(MagFuseSel.FUSE_MAG_ANCHORED)
        heading = self.get_heading()
        self.delay_sim_time(30, reason="ground yaw to walk if unanchored")
        self.assert_mag_fusion_selection(MagFuseSel.FUSE_MAG_ANCHORED)
        self.assert_heading(heading, accuracy=5, heading_source='VFR_HUD')
        self.arm_vehicle()
        self.delay_sim_time(1, reason="yaw anchor to release")
        self.assert_mag_fusion_selection(MagFuseSel.FUSE_MAG)
        self.disarm_vehicle()
        self.delay_sim_time(1, reason="yaw anchor to re-engage")
        self.assert_mag_fusion_selection(MagFuseSel.FUSE_MAG_ANCHORED)
