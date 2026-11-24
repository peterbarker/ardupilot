'''
Copter autotest: GuidedSubModeChange

AP_FLAKE8_CLEAN
'''

from pymavlink import mavutil

from vehicle_test_suite import MAV_POS_TARGET_TYPE_MASK
from vehicle_test_suite import NotAchievedException

from test_discovery import TestGroup
from test_discovery import testcase


class GuidedSubModeChange(TestGroup):
    def fly_guided_stop(self,
                        timeout=20,
                        groundspeed_tolerance=0.05,
                        climb_tolerance=0.01):
        """stop the vehicle moving in guided mode"""
        self.progress("Stopping vehicle")
        tstart = self.get_sim_time()
        # send a position-control command
        self.mav.mav.set_position_target_local_ned_send(
            0, # timestamp
            1, # target system_id
            1, # target component id
            mavutil.mavlink.MAV_FRAME_BODY_NED,
            MAV_POS_TARGET_TYPE_MASK.POS_ONLY | MAV_POS_TARGET_TYPE_MASK.LAST_BYTE, # mask specifying use-only-x-y-z
            0, # x
            0, # y
            0, # z
            0, # vx
            0, # vy
            0, # vz
            0, # afx
            0, # afy
            0, # afz
            0, # yaw
            0, # yawrate
        )
        while True:
            if self.get_sim_time_cached() - tstart > timeout:
                raise NotAchievedException("Vehicle did not stop")
            m = self.assert_receive_message('VFR_HUD')
            print("%s" % str(m))
            if (m.groundspeed < groundspeed_tolerance and
                    m.climb < climb_tolerance):
                break

    def test_guided_local_position_target(self, x, y, z_up):
        """ Check target position being received by vehicle """
        # set POSITION_TARGET_LOCAL_NED message rate using SET_MESSAGE_INTERVAL
        self.progress("Setting local target in NED: (%f, %f, %f)" % (x, y, -z_up))
        self.progress("Setting rate to 1 Hz")
        self.set_message_rate_hz(mavutil.mavlink.MAVLINK_MSG_ID_POSITION_TARGET_LOCAL_NED, 1)

        # mask specifying use only xyz
        target_typemask = MAV_POS_TARGET_TYPE_MASK.POS_ONLY

        # set position target
        self.mav.mav.set_position_target_local_ned_send(
            0, # timestamp
            1, # target system_id
            1, # target component id
            mavutil.mavlink.MAV_FRAME_LOCAL_NED,
            target_typemask | MAV_POS_TARGET_TYPE_MASK.LAST_BYTE,
            x, # x
            y, # y
            -z_up, # z
            0, # vx
            0, # vy
            0, # vz
            0, # afx
            0, # afy
            0, # afz
            0, # yaw
            0, # yawrate
        )
        m = self.assert_receive_message('POSITION_TARGET_LOCAL_NED', timeout=2)
        self.progress("Received local target: %s" % str(m))

        if not (m.type_mask == (target_typemask | MAV_POS_TARGET_TYPE_MASK.LAST_BYTE) or m.type_mask == target_typemask):
            raise NotAchievedException("Did not receive proper mask: expected=%u or %u, got=%u" %
                                       ((target_typemask | MAV_POS_TARGET_TYPE_MASK.LAST_BYTE), target_typemask, m.type_mask))

        if x - m.x > 0.1:
            raise NotAchievedException("Did not receive proper target position x: wanted=%f got=%f" % (x, m.x))

        if y - m.y > 0.1:
            raise NotAchievedException("Did not receive proper target position y: wanted=%f got=%f" % (y, m.y))

        if z_up - (-m.z) > 0.1:
            raise NotAchievedException("Did not receive proper target position z: wanted=%f got=%f" % (z_up, -m.z))

    def test_position_target_message_mode(self):
        " Ensure that POSITION_TARGET_LOCAL_NED messages are sent in Guided Mode only "
        self.hover()
        self.change_mode('LOITER')
        self.progress("Setting POSITION_TARGET_LOCAL_NED message rate to 10Hz")
        self.set_message_rate_hz(mavutil.mavlink.MAVLINK_MSG_ID_POSITION_TARGET_LOCAL_NED, 10)

        self.assert_not_receiving_message('POSITION_TARGET_LOCAL_NED')

        self.progress("Did not receive any POSITION_TARGET_LOCAL_NED message in LOITER mode. Success")

    @testcase(bucket="CopterTests2c")
    def GuidedSubModeChange(self):
        """"Ensure we can move around in guided after a takeoff command."""

        '''start by disabling GCS failsafe, otherwise we immediately disarm
        due to (apparently) not receiving traffic from the GCS for
        too long.  This is probably a function of --speedup'''
        self.set_parameters({
            "FS_GCS_ENABLE": 0,
            "DISARM_DELAY": 0, # until traffic problems are fixed
        })
        self.change_mode("GUIDED")
        self.wait_ready_to_arm()
        self.arm_vehicle()

        self.user_takeoff(alt_min=10)

        self.start_subtest("yaw through absolute angles using MAV_CMD_CONDITION_YAW")
        self.guided_achieve_heading(45, timeout=60)
        self.guided_achieve_heading(135, timeout=60)

        self.start_subtest("move the vehicle using set_position_target_global_int")
        # the following numbers are 5-degree-latitude and 5-degrees
        # longitude - just so that we start to really move a lot.
        self.fly_guided_move_global_relative_alt(5, 5, 10)

        self.start_subtest("move the vehicle using MAVLINK_MSG_ID_SET_POSITION_TARGET_LOCAL_NED")
        self.fly_guided_stop(groundspeed_tolerance=0.1)
        self.fly_guided_move_local(5, 5, 10)

        self.start_subtest("Checking that WP_YAW_BEHAVIOUR 0 works")
        self.set_parameter('WP_YAW_BEHAVIOR', 0)
        self.delay_sim_time(2, reason="parameter change to take effect")
        orig_heading = self.get_heading()
        self.fly_guided_move_local(5, 0, 10)
        # ensure our heading hasn't changed:
        self.assert_heading(orig_heading, accuracy=2)
        self.fly_guided_move_local(0, 5, 10)
        # ensure our heading hasn't changed:
        self.assert_heading(orig_heading, accuracy=2)

        self.start_subtest("Check target position received by vehicle using SET_MESSAGE_INTERVAL")
        self.test_guided_local_position_target(5, 5, 10)
        self.test_guided_local_velocity_target(2, 2, 1)
        self.test_position_target_message_mode()

        self.do_RTL()
