'''
Copter autotest: ScriptMountDriver

AP_FLAKE8_CLEAN
'''

from pymavlink import mavutil

from vehicle_test_suite import NotAchievedException

from test_discovery import TestGroup
from test_discovery import testcase


class ScriptMountDriver(TestGroup):
    @testcase(bucket="CopterTests2a")
    def ScriptMountDriver(self):
        '''test scripting mount driver with all modes and camera'''
        self.context_push()

        self.set_parameters({
            "SCR_ENABLE": 1,
            "MNT1_TYPE": 9,
            "MNT1_PITCH_MIN": -45,
            "MNT1_PITCH_MAX": 45,
        })
        self.reboot_sitl()

        self.install_example_script_context('mount-driver.lua')
        self.context_collect('STATUSTEXT')
        self.reboot_sitl()

        self.wait_statustext("MountDriver: started", check_context=True, timeout=30)
        self.wait_ready_to_arm()

        self.takeoff(20, mode='GUIDED')

        # test RETRACT mode - exercises angle_converted path
        self.start_subtest("RETRACT mode")
        retract_pitch = -15
        self.set_parameter("MNT1_RETRACT_Y", retract_pitch)
        self.run_cmd(
            mavutil.mavlink.MAV_CMD_DO_MOUNT_CONTROL,
            p7=mavutil.mavlink.MAV_MOUNT_MODE_RETRACT,
        )
        self.test_mount_pitch(retract_pitch, 1, mavutil.mavlink.MAV_MOUNT_MODE_RETRACT)

        # test NEUTRAL mode - exercises angle_converted path
        self.start_subtest("NEUTRAL mode")
        neutral_pitch = -10
        self.set_parameter("MNT1_NEUTRAL_Y", neutral_pitch)
        self.run_cmd(
            mavutil.mavlink.MAV_CMD_DO_MOUNT_CONTROL,
            p7=mavutil.mavlink.MAV_MOUNT_MODE_NEUTRAL,
        )
        self.test_mount_pitch(neutral_pitch, 1, mavutil.mavlink.MAV_MOUNT_MODE_NEUTRAL)

        # test MAVLINK_TARGETING with angle
        self.start_subtest("MAVLINK_TARGETING")
        self.run_cmd(
            mavutil.mavlink.MAV_CMD_DO_MOUNT_CONTROL,
            p1=20,  # pitch
            p2=0,   # roll
            p3=0,   # yaw
            p7=mavutil.mavlink.MAV_MOUNT_MODE_MAVLINK_TARGETING,
        )
        self.test_mount_pitch(20, 1, mavutil.mavlink.MAV_MOUNT_MODE_MAVLINK_TARGETING)

        # test MAVLINK_TARGETING with rate
        self.start_subtest("MAVLINK_TARGETING rate")
        # start at pitch 0
        self.run_cmd(
            mavutil.mavlink.MAV_CMD_DO_GIMBAL_MANAGER_PITCHYAW,
            p1=0,   # pitch angle
            p2=0,   # yaw angle
        )
        self.test_mount_pitch(0, 5, mavutil.mavlink.MAV_MOUNT_MODE_MAVLINK_TARGETING)
        # send pitch rate of -30 deg/s for 2 seconds
        self.run_cmd(
            mavutil.mavlink.MAV_CMD_DO_GIMBAL_MANAGER_PITCHYAW,
            p1=float('nan'),   # pitch angle (NaN = use rate)
            p2=float('nan'),   # yaw angle (NaN = use rate)
            p3=-30,            # pitch rate deg/s
            p4=0,              # yaw rate deg/s
        )
        self.delay_sim_time(2, reason="mount to slew at rate to target angle")
        # expect pitch around -60
        _, mount_pitch, _, _ = self.get_mount_roll_pitch_yaw_deg()
        if abs(mount_pitch - (-60)) > 20:
            raise NotAchievedException(
                "Rate mode pitch incorrect: got=%f want=-60 (+/-20)" % mount_pitch)
        self.progress("Rate mode pitch correct: %f degrees (~-60)" % mount_pitch)

        # test GPS_POINT (ROI)
        self.start_subtest("GPS_POINT (ROI)")
        takeoff_loc = self.get_location()
        t = self.offset_location_ne(takeoff_loc, 20, 0)
        self.run_cmd_int(
            mavutil.mavlink.MAV_CMD_DO_SET_ROI_LOCATION,
            p5=int(t.lat * 1e7),
            p6=int(t.lng * 1e7),
            p7=0,
            frame=mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
        )
        # We took off to 20m and the target is 20m North, so pitch should be around -45 degrees
        self.test_mount_pitch(-45, 5, mavutil.mavlink.MAV_MOUNT_MODE_GPS_POINT)

        # Reposition the aircraft 20m South of the takeoff location,
        # so it's now 40m from the target, and pitch should be around
        # -27 degrees
        t = self.offset_location_ne(takeoff_loc, -20, 0)
        self.send_set_position_target_global_int(int(t.lat * 1e7), int(t.lng * 1e7), 20)
        self.test_mount_pitch(-27, 5, mavutil.mavlink.MAV_MOUNT_MODE_GPS_POINT, constrained=False)

        # test HOME_LOCATION
        self.start_subtest("HOME_LOCATION")
        self.run_cmd(
            mavutil.mavlink.MAV_CMD_DO_MOUNT_CONTROL,
            p7=mavutil.mavlink.MAV_MOUNT_MODE_HOME_LOCATION,
        )
        # We are 20m South of home and at 20m altitude, so pitch should be around -45 degrees
        self.test_mount_pitch(-45, 5, mavutil.mavlink.MAV_MOUNT_MODE_HOME_LOCATION, constrained=False)

        # Reposition over home again, pitch should move to -90 degrees
        self.send_set_position_target_global_int(int(takeoff_loc.lat * 1e7), int(takeoff_loc.lng * 1e7), 20)
        self.test_mount_pitch(-90, 5, mavutil.mavlink.MAV_MOUNT_MODE_HOME_LOCATION, constrained=False)

        self.do_RTL()
        self.context_pop()
        self.reboot_sitl()
