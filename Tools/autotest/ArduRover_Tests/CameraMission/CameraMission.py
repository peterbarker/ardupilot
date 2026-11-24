'''
Rover autotest: CameraMission

AP_FLAKE8_CLEAN
'''

from vehicle_test_suite import NotAchievedException

from test_discovery import TestGroup
from test_discovery import testcase


class CameraMission(TestGroup):
    @testcase(bucket="Rover")
    def CameraMission(self):
        '''Test Camera Mission Items'''
        self.set_parameter("CAM1_TYPE", 1) # Camera with servo trigger
        self.reboot_sitl() # needed for CAM1_TYPE to take effect
        self.load_mission("rover-camera-mission.txt")
        self.wait_ready_to_arm()
        self.change_mode("AUTO")
        self.wait_ready_to_arm()
        self.arm_vehicle()
        prev_cf = None
        while True:
            cf = self.assert_receive_message('CAMERA_FEEDBACK')
            if prev_cf is None:
                prev_cf = cf
                continue
            dist_travelled = self.get_distance_int(prev_cf, cf)
            prev_cf = cf
            mc = self.mav.messages.get("MISSION_CURRENT", None)
            if mc is None:
                continue
            elif mc.seq == 2:
                expected_distance = 2
            elif mc.seq == 4:
                expected_distance = 5
            elif mc.seq == 5:
                break
            else:
                continue
            self.progress("Expected distance %f got %f" %
                          (expected_distance, dist_travelled))
            error = abs(expected_distance - dist_travelled)
            # Rover moves at ~5m/s; we appear to do something at
            # 5Hz, so we do see over a meter of error!
            max_error = 1.5
            if error > max_error:
                raise NotAchievedException("Camera distance error: %f (%f)" %
                                           (error, max_error))

        self.disarm_vehicle()
