'''
Rover autotest: JSONLockstep

AP_FLAKE8_CLEAN
'''

import time

from vehicle_test_suite import NotAchievedException

from test_discovery import TestGroup
from test_discovery import testcase


class JSONLockstep(TestGroup):
    @testcase(bucket="Rover")
    def JSONLockstep(self):
        '''check SITL runs in lockstep with a JSON physics backend'''
        # physics time far from zero, as a backend sending wall-clock time
        start_time = 1760000000.0
        with self.JSON_physics(start_time=start_time, dt=0.0025) as physics:
            self.wait_ready_to_arm()

            # SITL time is physics time, counted from the first frame
            self.JSON_assert_elapsed_equal((0, start_time, start_time), self.JSON_sim_and_physics_time(physics))

            # SITL matches its frame rate to the physics step
            frame_rate = self.JSON_frame_rate(physics)
            if abs(frame_rate - 400) > 2:
                raise NotAchievedException("frame_rate %.1f, expected 400" % frame_rate)

            # SITL waits for its physics, only resending the frame about once a second
            physics.paused = True
            packets = physics.packets
            time.sleep(3)
            # count before resuming, so resumed frames are not counted
            sent = physics.packets - packets
            physics.paused = False
            if sent > 5:
                raise NotAchievedException("SITL sent %u frames without physics" % sent)
            self.wait_heartbeat()

            # servo outputs reach the physics backend
            self.change_mode('MANUAL')
            self.arm_vehicle()
            self.set_rc(3, 1700)
            self.wait_servo_channel_in_range(3, 1650, 1750, timeout=5)
            # while the throttle slews the backend may already hold a newer
            # output than the message, so wait for the two to agree
            tstart = self.get_sim_time()
            while True:
                servo3 = self.assert_receive_message('SERVO_OUTPUT_RAW').servo3_raw
                if physics.pwm[2] == servo3:
                    break
                if self.get_sim_time_cached() - tstart > 5:
                    raise NotAchievedException("physics got throttle %u, expected %u" % (physics.pwm[2], servo3))
            self.set_rc(3, 1500)
            self.disarm_vehicle()
