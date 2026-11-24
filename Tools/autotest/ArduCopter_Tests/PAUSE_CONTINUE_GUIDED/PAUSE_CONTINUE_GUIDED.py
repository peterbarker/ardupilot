'''
Copter autotest: PAUSE_CONTINUE_GUIDED

AP_FLAKE8_CLEAN
'''

from pymavlink import mavutil

from vehicle_test_suite import MAV_POS_TARGET_TYPE_MASK
from vehicle_test_suite import AltFrame

from test_discovery import TestGroup
from test_discovery import testcase


class PAUSE_CONTINUE_GUIDED(TestGroup):
    @testcase(bucket="CopterTests2c")
    def PAUSE_CONTINUE_GUIDED(self):
        '''Test MAV_CMD_DO_PAUSE_CONTINUE in GUIDED mode'''
        self.start_subtest("Started test for Pause/Continue in GUIDED mode with LOCATION!")
        self.change_mode(mode="GUIDED")
        self.wait_ready_to_arm()
        self.arm_vehicle()
        self.set_parameter(name="GUID_TIMEOUT", value=120)
        self.user_takeoff(alt_min=30)

        # send vehicle to global position target
        target_alt = 30
        location = self.offset_location_ne(self.home_position_as_location(), 300, 0)
        location.set_alt_m(target_alt, AltFrame.ABOVE_HOME)
        target_typemask = MAV_POS_TARGET_TYPE_MASK.POS_ONLY
        self.mav.mav.set_position_target_global_int_send(
            0, # timestamp
            1, # target system_id
            1, # target component id
            mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT, # relative altitude frame
            target_typemask | MAV_POS_TARGET_TYPE_MASK.LAST_BYTE, # target typemask as pos only
            int(location.lat * 1e7), # lat
            int(location.lng * 1e7), # lon
            target_alt, # alt
            0, # vx
            0, # vy
            0, # vz
            0, # afx
            0, # afy
            0, # afz
            0, # yaw
            0) # yawrate

        self.wait_distance_to_home(distance_min=100, distance_max=150, timeout=120)
        self.send_pause_command()
        self.wait_groundspeed(speed_min=0, speed_max=1, minimum_duration=5)
        self.send_resume_command()
        self.wait_location(loc=location, timeout=120)

        self.end_subtest("Ended test for Pause/Continue in GUIDED mode with LOCATION!")
        self.start_subtest("Started test for Pause/Continue in GUIDED mode with DESTINATION!")
        self.guided_achieve_heading(heading=270)

        # move vehicle on x direction
        location = self.offset_location_ne(location=self.get_location(), metres_north=0, metres_east=-300)
        self.mav.mav.set_position_target_global_int_send(
            0, # system time in milliseconds
            1, # target system
            1, # target component
            mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT, # coordinate frame MAV_FRAME_BODY_NED
            MAV_POS_TARGET_TYPE_MASK.POS_ONLY, # type mask (pos only)
            int(location.lat*1e7), # position x
            int(location.lng*1e7), # position y
            30, # position z
            0, # velocity x
            0, # velocity y
            0, # velocity z
            0, # accel x
            0, # accel y
            0, # accel z
            0, # yaw
            0) # yaw rate

        self.wait_location(loc=location, accuracy=200, timeout=120)
        self.send_pause_command()
        self.wait_groundspeed(speed_min=0, speed_max=1, minimum_duration=5)
        self.send_resume_command()
        self.wait_location(loc=location, timeout=120)

        self.end_subtest("Ended test for Pause/Continue in GUIDED mode with DESTINATION!")
        self.start_subtest("Started test for Pause/Continue in GUIDED mode with VELOCITY!")

        # give velocity command
        vx, vy, vz_up = (5, 5, 0)
        self.test_guided_local_velocity_target(vx=vx, vy=vy, vz_up=vz_up, timeout=10)

        self.wait_for_local_velocity(vx=vx, vy=vy, vz_up=vz_up, timeout=10)
        self.send_pause_command()
        self.wait_for_local_velocity(vx=0, vy=0, vz_up=0, timeout=10)
        self.send_resume_command()
        self.wait_for_local_velocity(vx=vx, vy=vy, vz_up=vz_up, timeout=10)
        self.test_guided_local_velocity_target(vx=0, vy=0, vz_up=0, timeout=10)
        self.wait_for_local_velocity(vx=0, vy=0, vz_up=0, timeout=10)

        self.end_subtest("Ended test for Pause/Continue in GUIDED mode with VELOCITY!")
        self.start_subtest("Started test for Pause/Continue in GUIDED mode with ACCELERATION!")

        # give acceleration command
        ax, ay, az_up = (1, 1, 0)
        target_typemask = (MAV_POS_TARGET_TYPE_MASK.POS_IGNORE | MAV_POS_TARGET_TYPE_MASK.VEL_IGNORE |
                           MAV_POS_TARGET_TYPE_MASK.YAW_IGNORE | MAV_POS_TARGET_TYPE_MASK.YAW_RATE_IGNORE)
        self.mav.mav.set_position_target_local_ned_send(
            0, # timestamp
            1, # target system_id
            1, # target component id
            mavutil.mavlink.MAV_FRAME_LOCAL_NED,
            target_typemask | MAV_POS_TARGET_TYPE_MASK.LAST_BYTE,
            0, # x
            0, # y
            0, # z
            0, # vx
            0, # vy
            0, # vz
            ax, # afx
            ay, # afy
            -az_up, # afz
            0, # yaw
            0, # yawrate
        )

        self.wait_for_local_velocity(vx=5, vy=5, vz_up=0, timeout=10)
        self.send_pause_command()
        self.wait_for_local_velocity(vx=0, vy=0, vz_up=0, timeout=10)
        self.send_resume_command()
        self.wait_for_local_velocity(vx=5, vy=5, vz_up=0, timeout=10)
        self.test_guided_local_velocity_target(vx=0, vy=0, vz_up=0, timeout=10)
        self.wait_for_local_velocity(vx=0, vy=0, vz_up=0, timeout=10)
        self.end_subtest("Ended test for Pause/Continue in GUIDED mode with ACCELERATION!")

        # start pause/continue subtest with posvelaccel
        self.start_subtest("Started test for Pause/Continue in GUIDED mode with POSITION and VELOCITY and ACCELERATION!")
        self.guided_achieve_heading(heading=0)

        # give posvelaccel command
        x, y, z_up = (-300, 0, 30)
        target_typemask = (MAV_POS_TARGET_TYPE_MASK.YAW_IGNORE | MAV_POS_TARGET_TYPE_MASK.YAW_RATE_IGNORE)
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

        self.wait_distance_to_local_position(local_position=(x, y, -z_up), distance_min=400, distance_max=450, timeout=120)
        self.send_pause_command()
        self.wait_for_local_velocity(0, 0, 0, timeout=10)
        self.send_resume_command()
        self.wait_distance_to_local_position(local_position=(x, y, -z_up), distance_min=0, distance_max=10, timeout=120)

        self.end_subtest("Ended test for Pause/Continue in GUIDED mode with POSITION and VELOCITY and ACCELERATION!")
        self.do_RTL(timeout=120)
