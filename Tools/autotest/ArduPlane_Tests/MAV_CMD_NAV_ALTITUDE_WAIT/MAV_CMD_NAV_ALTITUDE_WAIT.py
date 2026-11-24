'''
Plane autotest: MAV_CMD_NAV_ALTITUDE_WAIT

AP_FLAKE8_CLEAN
'''

from pymavlink import mavutil

from vehicle_test_suite import NotAchievedException

from test_discovery import TestGroup
from test_discovery import testcase


class MAV_CMD_NAV_ALTITUDE_WAIT(TestGroup):
    @testcase(bucket="PlaneTests1b")
    def MAV_CMD_NAV_ALTITUDE_WAIT(self):
        '''test MAV_CMD_NAV_ALTITUDE_WAIT mission item, wiggling only'''

        # Load a single waypoint
        self.upload_simple_relhome_mission([
            self.create_MISSION_ITEM_INT(
                mavutil.mavlink.MAV_CMD_NAV_ALTITUDE_WAIT,
                p1=1000, # 1000m alt threshold, this should not trigger
                p2=10, # 10m/s descent rate, this should not trigger
                p3=10 # servo wiggle every 10 seconds
            )
        ])

        # Set initial conditions for servo wiggle testing
        servo_wiggled = {1: False, 2: False, 4: False}

        def look_for_wiggle(mav, m):
            if m.get_type() == 'SERVO_OUTPUT_RAW':
                # Throttle must be zero
                if m.servo3_raw != 1000:
                    raise NotAchievedException(
                        "Throttle must be 0 in altitude wait, got %f" % m.servo3_raw)

                # Check if all servos wiggle
                if m.servo1_raw != 1500:
                    servo_wiggled[1] = True
                if m.servo2_raw != 1500:
                    servo_wiggled[2] = True
                if m.servo4_raw != 1500:
                    servo_wiggled[4] = True

        # Start mission
        self.change_mode('AUTO')
        self.wait_ready_to_arm()
        self.arm_vehicle()

        # Check outputs
        self.context_push()
        self.install_message_hook_context(look_for_wiggle)

        # Wait for a bit to let message hook sample
        self.delay_sim_time(60, reason="message hook to sample servo outputs")

        self.context_pop()

        # If the mission item completes as there is no other waypoints we will end up in RTL
        if not self.mode_is('AUTO'):
            raise NotAchievedException("Must still be in AUTO")

        # Raise error if not all servos have wiggled
        if not all(servo_wiggled.values()):
            raise NotAchievedException("Not all servos have moved within the test frame")

        self.disarm_vehicle()
