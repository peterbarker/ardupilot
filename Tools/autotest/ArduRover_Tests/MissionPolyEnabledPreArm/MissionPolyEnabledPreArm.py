'''
Rover autotest: MissionPolyEnabledPreArm

AP_FLAKE8_CLEAN
'''

from pymavlink import mavutil

from test_discovery import TestGroup
from test_discovery import testcase


class MissionPolyEnabledPreArm(TestGroup):
    @testcase(bucket="Rover")
    def MissionPolyEnabledPreArm(self):
        '''check Polygon porearm checks'''
        self.set_parameters({
            'FENCE_ENABLE': 1,
        })
        self.progress("Ensure that we can arm if polyfence is enabled but we have no polyfence")
        self.assert_parameter_value('FENCE_TYPE', 6)
        self.wait_ready_to_arm()
        self.reboot_sitl()
        self.wait_ready_to_arm()

        self.progress("Ensure we can arm when we have an inclusion fence we are inside of")
        here = self.get_location()
        self.upload_fences_from_locations([
            (mavutil.mavlink.MAV_CMD_NAV_FENCE_POLYGON_VERTEX_INCLUSION, [
                # over the top of the vehicle
                self.offset_location_ne(here, -50, -50), # bl
                self.offset_location_ne(here, -50, 50), # br
                self.offset_location_ne(here, 50, 50), # tr
                self.offset_location_ne(here, 50, -50), # tl,
            ]),
        ])
        self.wait_ready_to_arm()

        self.reboot_sitl()
        self.wait_ready_to_arm()

        self.progress("Ensure we can't arm when we are in breach of a polyfence")
        self.clear_fence()

        self.progress("Now create a fence we are in breach of")
        here = self.get_location()
        self.upload_fences_from_locations([
            (mavutil.mavlink.MAV_CMD_NAV_FENCE_POLYGON_VERTEX_INCLUSION, [
                # over the top of the vehicle
                self.offset_location_ne(here, 20, 20), # bl
                self.offset_location_ne(here, 20, 50), # br
                self.offset_location_ne(here, 50, 50), # tr
                self.offset_location_ne(here, 50, 20), # tl,
            ]),
        ])

        self.assert_prearm_failure('Vehicle breaching Polygon fence', other_prearm_failures_fatal=False)
        self.reboot_sitl()

        self.assert_prearm_failure('Vehicle breaching Polygon fence', other_prearm_failures_fatal=False, timeout=120)

        self.progress("Ensure we can arm when a polyfence fence is cleared when we've previously been in breach")
        self.clear_fence()
        self.wait_ready_to_arm()

        self.upload_fences_from_locations([
            (mavutil.mavlink.MAV_CMD_NAV_FENCE_POLYGON_VERTEX_INCLUSION, [
                # over the top of the vehicle
                self.offset_location_ne(here, 20, 20), # bl
                self.offset_location_ne(here, 20, 50), # br
                self.offset_location_ne(here, 50, 50), # tr
                self.offset_location_ne(here, 50, 20), # tl,
            ]),
        ])
        self.reboot_sitl()
        self.assert_prearm_failure('Vehicle breaching Polygon fence', other_prearm_failures_fatal=False, timeout=120)
        self.clear_fence()
        self.wait_ready_to_arm()

        self.progress("Ensure we can arm after clearing polygon fence type enabled")
        self.upload_fences_from_locations([
            (mavutil.mavlink.MAV_CMD_NAV_FENCE_POLYGON_VERTEX_INCLUSION, [
                # over the top of the vehicle
                self.offset_location_ne(here, 20, 20), # bl
                self.offset_location_ne(here, 20, 50), # br
                self.offset_location_ne(here, 50, 50), # tr
                self.offset_location_ne(here, 50, 20), # tl,
            ]),
        ])
        self.assert_prearm_failure('Vehicle breaching Polygon fence', other_prearm_failures_fatal=False, timeout=120)
        self.set_parameter('FENCE_TYPE', 2)
        self.wait_ready_to_arm()
        self.set_parameter('FENCE_TYPE', 6)
        self.assert_prearm_failure('Vehicle breaching Polygon fence', other_prearm_failures_fatal=False, timeout=120)
