'''
Copter autotest: MAV_CMD_DO_MOUNT_CONTROL

AP_FLAKE8_CLEAN
'''

from pymavlink import mavutil

from test_discovery import TestGroup
from test_discovery import testcase


class MAV_CMD_DO_MOUNT_CONTROL(TestGroup):
    @testcase(bucket="CopterTests1e")
    def MAV_CMD_DO_MOUNT_CONTROL(self):
        '''test MAV_CMD_DO_MOUNT_CONTROL mavlink command'''

        # setup mount parameters
        self.context_push()
        self.setup_servo_mount()
        self.reboot_sitl() # to handle MNT_TYPE changing

        takeoff_loc = self.get_location()

        self.takeoff(20, mode='GUIDED')
        self.guided_achieve_heading(315, direction=1, accuracy=1)

        self.run_cmd(
            mavutil.mavlink.MAV_CMD_DO_MOUNT_CONTROL,
            p7=mavutil.mavlink.MAV_MOUNT_MODE_RETRACT,
        )
        self.run_cmd_int(
            mavutil.mavlink.MAV_CMD_DO_MOUNT_CONTROL,
            p7=mavutil.mavlink.MAV_MOUNT_MODE_RETRACT,
        )

        for method in self.run_cmd, self.run_cmd_int:
            self.start_subtest("MAV_MOUNT_MODE_GPS_POINT")

            self.progress("start=%s" % str(takeoff_loc))
            t = self.offset_location_ne(takeoff_loc, 20, 0)
            self.progress("targeting=%s" % str(t))

            # this command is *weird* as the lat/lng is *always* 1e7,
            # even when transported via COMMAND_LONG!
            x = int(t.lat * 1e7)
            y = int(t.lng * 1e7)
            method(
                mavutil.mavlink.MAV_CMD_DO_MOUNT_CONTROL,
                p4=0,  # this is a relative altitude!
                p5=x,
                p6=y,
                p7=mavutil.mavlink.MAV_MOUNT_MODE_GPS_POINT,
            )
            self.test_mount_pitch(-45, 5, mavutil.mavlink.MAV_MOUNT_MODE_GPS_POINT)
            self.neutralise_gimbal()

            self.start_subtest("MAV_MOUNT_MODE_HOME_LOCATION")
            method(
                mavutil.mavlink.MAV_CMD_DO_MOUNT_CONTROL,
                p7=mavutil.mavlink.MAV_MOUNT_MODE_HOME_LOCATION,
            )
            self.test_mount_pitch(-90, 5, mavutil.mavlink.MAV_MOUNT_MODE_HOME_LOCATION)
            self.neutralise_gimbal()

            # try an invalid mount mode.  Note that this is asserting we
            # are receiving a result code which is actually incorrect;
            # this should be MAV_RESULT_DENIED
            self.start_subtest("Invalid mode")
            method(
                mavutil.mavlink.MAV_CMD_DO_MOUNT_CONTROL,
                p7=87,
                want_result=mavutil.mavlink.MAV_RESULT_FAILED,
            )

            self.start_subtest("MAV_MOUNT_MODE_MAVLINK_TARGETING")
            r = 15
            p = 20
            y = 30
            method(
                mavutil.mavlink.MAV_CMD_DO_MOUNT_CONTROL,
                p1=p,
                p2=r,
                p3=y,
                p7=mavutil.mavlink.MAV_MOUNT_MODE_MAVLINK_TARGETING,
            )
            self.delay_sim_time(2, reason="mount to reach target angles")
            self.assert_mount_rpy(r, p, y)
            self.neutralise_gimbal()

            self.start_subtest("MAV_MOUNT_MODE_RC_TARGETING")
            method(
                mavutil.mavlink.MAV_CMD_DO_MOUNT_CONTROL,
                p7=mavutil.mavlink.MAV_MOUNT_MODE_RC_TARGETING,
            )
            self.test_mount_rc_targetting()

            self.start_subtest("MAV_MOUNT_MODE_RETRACT")
            self.context_push()
            retract_r = 13
            retract_p = 23
            retract_y = 33
            self.set_parameters({
                "MNT1_RETRACT_X": retract_r,
                "MNT1_RETRACT_Y": retract_p,
                "MNT1_RETRACT_Z": retract_y,
            })
            method(
                mavutil.mavlink.MAV_CMD_DO_MOUNT_CONTROL,
                p7=mavutil.mavlink.MAV_MOUNT_MODE_RETRACT,
            )
            self.delay_sim_time(3, reason="mount to reach retract position")
            self.assert_mount_rpy(retract_r, retract_p, retract_y)
            self.context_pop()

        self.do_RTL()

        self.context_pop()
        self.reboot_sitl()
