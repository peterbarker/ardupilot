'''
Copter autotest: ScriptMountAllModes

AP_FLAKE8_CLEAN
'''

from pymavlink import mavutil

from test_discovery import TestGroup
from test_discovery import testcase


class ScriptMountAllModes(TestGroup):
    @testcase(bucket="CopterTests1c")
    def ScriptMountAllModes(self):
        '''test location from all mount modes using the scripting backend'''

        # enable scripting and set mount type to scripting
        self.set_parameters({
            "SCR_ENABLE": 1,
            "MNT1_TYPE": 9,
        })
        self.reboot_sitl()

        # install get-target-location-mount-backend.lua script
        self.install_example_script_context('get-target-location-mount-backend.lua')
        self.reboot_sitl()
        self.takeoff(50, mode='GUIDED')

        self.context_collect("STATUSTEXT")

        self.start_subtest("Test RETRACT (mode 0)")
        self.run_cmd(
            mavutil.mavlink.MAV_CMD_DO_MOUNT_CONTROL,
            p7=mavutil.mavlink.MAV_MOUNT_MODE_RETRACT,
        )
        self.wait_statustext("Mode 0: No target", check_context=True)

        self.start_subtest("Test NEUTRAL (mode 1)")
        self.run_cmd(
            mavutil.mavlink.MAV_CMD_DO_MOUNT_CONTROL,
            p7=mavutil.mavlink.MAV_MOUNT_MODE_NEUTRAL,
        )
        self.wait_statustext("Mode 1: No target", check_context=True)

        self.start_subtest("Test MAVLINK_TARGETING (mode 2)")
        self.run_cmd(
            mavutil.mavlink.MAV_CMD_DO_MOUNT_CONTROL,
            p7=mavutil.mavlink.MAV_MOUNT_MODE_MAVLINK_TARGETING,
        )
        self.wait_statustext("Mode 2: No target", check_context=True)

        self.start_subtest("Test RC_TARGETING (mode 3)")
        self.run_cmd(
            mavutil.mavlink.MAV_CMD_DO_MOUNT_CONTROL,
            p7=mavutil.mavlink.MAV_MOUNT_MODE_RC_TARGETING,
        )
        self.wait_statustext("Mode 3: No target", check_context=True)

        self.start_subtest("Test GPS_POINT (mode 4)")
        roi_lat = -35.363150
        roi_lng = 149.165320
        roi_alt = 580.0

        self.run_cmd_int(
            mavutil.mavlink.MAV_CMD_DO_SET_ROI_LOCATION,
            p5=int(roi_lat * 1e7),
            p6=int(roi_lng * 1e7),
            p7=roi_alt,
            frame=mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
        )

        expected_roi_lat = f"{roi_lat:.3f}"
        expected_roi_lng = f"{roi_lng:.3f}"

        self.wait_statustext(
            rf"Mode 4:.*{expected_roi_lat}.*{expected_roi_lng}",
            check_context=True,
            regex=True,
        )

        self.start_subtest("Test SYSID_TARGET (mode 5)")
        self.run_cmd(
            mavutil.mavlink.MAV_CMD_DO_SET_ROI_SYSID,
            p1=self.mav.source_system,
        )
        # Single-vehicle SITL, SYSID_TARGET has no target to track
        self.wait_statustext("Mode 5: No target", check_context=True)

        self.start_subtest("Test HOME_LOCATION (mode 6)")
        loc_home = self.poll_home_position()

        expected_lat = f"{loc_home.latitude/1e7:.6f}"
        expected_lng = f"{loc_home.longitude/1e7:.6f}"

        self.run_cmd(
            mavutil.mavlink.MAV_CMD_DO_MOUNT_CONTROL,
            p7=mavutil.mavlink.MAV_MOUNT_MODE_HOME_LOCATION,
        )

        self.wait_statustext(
            rf"Mode 6:.*{expected_lat}.*{expected_lng}",
            check_context=True,
            regex=True,
        )

        self.do_RTL()
