'''
QuadPlane autotest: TiltrotorVectoringRoll

AP_FLAKE8_CLEAN
'''

import operator

from vehicle_test_suite import NotAchievedException

from test_discovery import TestGroup
from test_discovery import testcase


class TiltrotorVectoringRoll(TestGroup):
    @testcase(bucket="QuadPlaneTests1c")
    def TiltrotorVectoringRoll(self):
        '''check tilt-vectoring drives the tilt servos the correct way for roll'''

        # A vectored-yaw tiltrotor uses the tilt servos for both yaw and
        # roll.  Tilting a rotor further forward reduces its vertical
        # thrust, so a right roll demand (right side down) must tilt the
        # left rotor back and the right rotor forward.  That is the
        # opposite of the tilt used for a right yaw demand, where the
        # left rotor goes forward.  Roll is scaled by sin(tilt) and yaw by
        # cos(tilt), so the roll term only shows up at large tilt angles.
        #
        # The vehicle is held on the ground where SITL pins it level with
        # zero body rates, so a stick demand produces a steady rate
        # controller output without the vehicle ever moving.

        # SERVO13 is k_tiltMotorLeft and SERVO12 is k_tiltMotorRight; a
        # higher output tilts that side further forward:
        LEFT_TILT_SERVO = 13
        RIGHT_TILT_SERVO = 12
        self.customise_SITL_commandline(
            [],
            model="quadplane-tilttrivec",
            wipe=True,
        )

        self.set_parameters({
            # FWD_THR gives the pilot direct control of the tilt angle in QACRO:
            "RC6_OPTION": 209,
            "Q_FWD_MANTHR_MAX": 100,
            # AIRMODE keeps the rate controllers running with the throttle closed,
            # so the vehicle can be held on the ground while it demands roll:
            "RC7_OPTION": 84,
            # vectoring uses sin(tilt) for roll and cos(tilt) for yaw, so a large
            # tilt angle makes the roll term the dominant one:
            "Q_TILT_MAX": 80,
        })
        self.context_set_message_rate_hz('SERVO_OUTPUT_RAW', 20)

        def tilt_offset():
            '''left tilt servo output minus right tilt servo output, in PWM'''
            m = self.assert_receive_message('SERVO_OUTPUT_RAW', timeout=5)
            left = getattr(m, "servo%u_raw" % LEFT_TILT_SERVO)
            right = getattr(m, "servo%u_raw" % RIGHT_TILT_SERVO)
            return left - right

        def wait_tilt_offset(minimum, timeout=10):
            '''wait for the tilt servos to split by minimum PWM, return signed offset'''
            tstart = self.get_sim_time()
            while True:
                offset = tilt_offset()
                if abs(offset) >= minimum:
                    self.progress("tilt offset %d" % offset)
                    return offset
                if self.get_sim_time_cached() - tstart > timeout:
                    raise NotAchievedException(
                        "tilt servos did not vector (offset=%d)" % offset)

        def wait_tilt_centred(maximum=15, timeout=10):
            '''wait for the tilt servos to come back together'''
            tstart = self.get_sim_time()
            while True:
                offset = tilt_offset()
                if abs(offset) <= maximum:
                    return
                if self.get_sim_time_cached() - tstart > timeout:
                    raise NotAchievedException(
                        "tilt servos did not centre (offset=%d)" % offset)

        def check_tilt_offset(name, want_positive, minimum=25, duration=2):
            '''check the tilt servos stay split the expected way for duration'''
            wait_tilt_offset(minimum)
            sign = 1 if want_positive else -1
            offsets = []
            tstart = self.get_sim_time()
            while self.get_sim_time_cached() - tstart < duration:
                offsets.append(tilt_offset() * sign)
            self.progress("%s: tilt offsets %d..%d (want >= %d)" %
                          (name, min(offsets), max(offsets), minimum))
            if min(offsets) < minimum:
                raise NotAchievedException(
                    "%s: tilt vectored the wrong way (offset=%d)" %
                    (name, min(offsets) * sign))

        self.change_mode('QACRO')
        self.wait_ready_to_arm()
        self.zero_throttle()
        self.set_rc(6, 1000)  # rotors up
        self.arm_vehicle()
        self.set_rc(7, 2000)  # air mode on

        self.start_subtest("roll vectors the tilt servos with the rotors tilted")
        self.set_rc(6, 2000)  # rotors forward to Q_TILT_MAX
        # both servos sit at the tilt angle while roll and yaw are neutral:
        self.wait_servo_channel_value(LEFT_TILT_SERVO, 1850, comparator=operator.gt, timeout=20)
        self.wait_servo_channel_value(RIGHT_TILT_SERVO, 1850, comparator=operator.gt, timeout=20)
        wait_tilt_centred()

        # a right roll demand must tilt the left rotor back, so that the
        # left side lifts and the right side drops:
        self.set_rc(1, 2000)
        check_tilt_offset("right roll", want_positive=False)

        self.set_rc(1, 1500)
        wait_tilt_centred()

        self.set_rc(1, 1000)
        check_tilt_offset("left roll", want_positive=True)

        self.set_rc(1, 1500)
        wait_tilt_centred()

        self.start_subtest("rudder vectors the tilt servos with the rotors up")
        # with the rotors up sin(tilt) is zero, so no roll vectoring is
        # applied and this checks the yaw half of the same calculation
        self.set_rc(6, 1000)
        self.wait_servo_channel_value(LEFT_TILT_SERVO, 1150, comparator=operator.lt, timeout=20)
        self.wait_servo_channel_value(RIGHT_TILT_SERVO, 1150, comparator=operator.lt, timeout=20)

        # right rudder tilts the left rotor forward, which yaws the nose right
        self.set_rc(4, 2000)
        check_tilt_offset("right rudder", want_positive=True)

        self.set_rc(4, 1500)
        self.set_rc(7, 1000)  # air mode off
        # air mode has been running the motors up, so the vehicle still
        # believes it is flying:
        self.disarm_vehicle(force=True)
