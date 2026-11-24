'''
Copter autotest: MAV_CMD_DO_SET_ROI_WPNEXT_OFFSET

AP_FLAKE8_CLEAN
'''

from pymavlink import mavutil

from test_discovery import TestGroup
from test_discovery import testcase


class MAV_CMD_DO_SET_ROI_WPNEXT_OFFSET(TestGroup):
    @testcase(bucket="CopterTests1a")
    def MAV_CMD_DO_SET_ROI_WPNEXT_OFFSET(self):
        '''check the MAV_CMD_DO_SET_ROI_WPNEXT_OFFSET mission item'''
        # setup mount parameters
        self.setup_servo_mount()
        self.reboot_sitl() # to handle MNT_TYPE changing

        alt = 50

        loc = self.get_location()

        items = [
            self.mission_item_home(),
            self.mission_item_copter_takeoff(alt=alt),
        ]

        loc = self.offset_location_ne(loc, 100, 0)
        items.append(self.mission_item_do_cmd_roi_set_wpnext_offset(p=0))
        items.append(self.mission_item_waypoint(loc.lat, loc.lng, alt))

        loc = self.offset_location_ne(loc, 100, 0)
        items.append(self.mission_item_do_cmd_roi_set_wpnext_offset(p=-45))
        items.append(self.mission_item_waypoint(loc.lat, loc.lng, alt))

        loc = self.offset_location_ne(loc, 100, 0)
        items.append(self.mission_item_do_cmd_roi_set_wpnext_offset(y=45))
        items.append(self.mission_item_waypoint(loc.lat, loc.lng, alt))

        # clear the ROI; the mount must return to its default mode
        loc = self.offset_location_ne(loc, 100, 0)
        items.append(self.create_MISSION_ITEM_INT(mavutil.mavlink.MAV_CMD_DO_SET_ROI_NONE))
        items.append(self.mission_item_waypoint(loc.lat, loc.lng, alt))

        # now RTL
        items.append(self.mission_item_rtl())

        self.renumber_mission_items(items)
        self.check_mission_upload_download(items)

        self.set_parameter('AUTO_OPTIONS', 3)
        self.wait_ready_to_arm()
        self.arm_vehicle()
        self.change_mode('AUTO')

        self.wait_current_waypoint(3)
        # the 30 here is because the vehicle is pitching down and the
        # gimbal is compensating
        self.wait_mount_roll_pitch_yaw_deg(r=0, p=30, y=0)

        self.wait_current_waypoint(5)
        # the -15 here is because the vehicle is pitching down and the
        # gimbal is compensating, and is being asked to pitch down 45 degrees
        self.wait_mount_roll_pitch_yaw_deg(r=0, p=-15, y=0)

        self.wait_current_waypoint(7)
        self.wait_mount_roll_pitch_yaw_deg(y=45)

        # the ROI should have been cleared now
        # default mode is RC_TARGETING with neutral RC input
        self.wait_current_waypoint(9)
        self.wait_mount_roll_pitch_yaw_deg(r=0, p=0, y=0)

        self.wait_disarmed()
