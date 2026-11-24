'''
Copter autotest: MountYawVehicleForMountROI

AP_FLAKE8_CLEAN
'''

from pymavlink import mavextra
from pymavlink import mavutil

from vehicle_test_suite import MAV_POS_TARGET_TYPE_MASK
from vehicle_test_suite import AltFrame
from vehicle_test_suite import Location

from test_discovery import TestGroup
from test_discovery import testcase


class MountYawVehicleForMountROI(TestGroup):
    @testcase(bucket="CopterTests1e")
    def MountYawVehicleForMountROI(self):
        '''Test Camera/Antenna Mount vehicle yawing for ROI'''
        self.set_parameter("MAV_GCS_SYSID", self.mav.source_system)
        yaw_servo = 7
        self.setup_servo_mount(yaw_servo=yaw_servo)
        self.reboot_sitl() # to handle MNT1_TYPE changing

        self.progress("checking ArduCopter yaw-aircraft-for-roi")
        self.takeoff(20, mode='GUIDED')

        m = self.assert_receive_message('VFR_HUD')
        self.progress("current heading %u" % m.heading)
        self.set_parameter("SERVO%u_FUNCTION" % yaw_servo, 0) # yaw
        self.progress("Waiting for check_servo_map to do its job")
        self.delay_sim_time(5, reason="servo map to update")
        self.progress("Pointing North")
        self.guided_achieve_heading(0, direction=1, accuracy=1)
        self.delay_sim_time(5, reason="heading to stabilise")
        start = self.get_location()
        (roi_lat, roi_lon) = mavextra.gps_offset(start.lat,
                                                 start.lng,
                                                 -100,
                                                 -100)
        roi_alt = 0
        self.progress("Using MAV_CMD_DO_SET_ROI")
        self.run_cmd(
            mavutil.mavlink.MAV_CMD_DO_SET_ROI,
            p5=roi_lat,
            p6=roi_lon,
            p7=roi_alt,
        )

        self.progress("Waiting for vehicle to point towards ROI")
        self.wait_heading(225, timeout=600, minimum_duration=2)

        # the following numbers are 1-degree-latitude and
        # 0-degrees longitude - just so that we start to
        # really move a lot.
        there = Location(1, 0, 0, AltFrame.ABOVE_HOME)

        self.progress("Starting to move")
        self.mav.mav.set_position_target_global_int_send(
            0, # timestamp
            1, # target system_id
            1, # target component id
            mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
            MAV_POS_TARGET_TYPE_MASK.POS_ONLY | MAV_POS_TARGET_TYPE_MASK.LAST_BYTE, # mask specifying use-only-lat-lon-alt
            there.lat, # lat
            there.lng, # lon
            there.get_alt_m(AltFrame.ABOVE_HOME), # alt
            0, # vx
            0, # vy
            0, # vz
            0, # afx
            0, # afy
            0, # afz
            0, # yaw
            0, # yawrate
        )

        self.progress("Starting to move changes the target")
        bearing = self.bearing_to(there)
        self.wait_heading(bearing, timeout=600, minimum_duration=2)

        self.run_cmd(
            mavutil.mavlink.MAV_CMD_DO_SET_ROI,
            p5=roi_lat,
            p6=roi_lon,
            p7=roi_alt,
        )

        self.progress("Wait for vehicle to point sssse due to moving")
        self.wait_heading(170, timeout=600, minimum_duration=1)

        self.do_RTL()
