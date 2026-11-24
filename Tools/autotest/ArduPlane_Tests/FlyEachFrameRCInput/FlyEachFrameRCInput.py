'''
Plane autotest: FlyEachFrameRCInput

AP_FLAKE8_CLEAN
'''

from pysim import vehicleinfo

from test_discovery import TestGroup
from test_discovery import testcase


class FlyEachFrameRCInput(TestGroup):
    @testcase(bucket="PlaneTests1b")
    def FlyEachFrameRCInput(self):
        '''Confirm each internal frame can be controlled with RC in the
        expected direction.

        FlyEachFrame flies AUTO missions, which never touch the sticks, so
        pilot-input direction (e.g. an RCn_REVERSED) is untested there.
        For every frame we check the autotest stick convention: elevator
        back (high RC2) pitches the nose up, aileron left (low RC1) banks
        left.  Frames whose model deliberately reverses the pilot's RC
        pitch/yaw input (models/plane-reverse-rc-pitch-yaw.parm) instead
        expect the opposite pitch response.'''
        vinfo = vehicleinfo.VehicleInfo()
        vinfo_options = vinfo.options[self.vehicleinfo_key()]
        known_broken_frames = {
            "plane-tailsitter": "unstable in hover; unflyable in cruise",
            "stratoblimp": "not expected to fly normally",
            "glider": "needs balloon lift",
            "plane-ice": "ICE engine needs starting; different takeoff regime",
            "quadplane-ice": "ICE engine needs starting; different takeoff regime",
            "quadplane-can": "needs CAN periph",
            "quadplane-copter_tailsitter": "tailsitter hover regime; nose-up attitude",
        }
        # frames whose model deliberately reverses the pilot's RC pitch/yaw
        # input (models/plane-reverse-rc-pitch-yaw.parm); the nose responds
        # opposite to the elevator stick.  RC roll input is never reversed.
        rc_pitch_reversed_frames = (
            "plane-elevrev",
            "plane-redundant",  # reversed for the Volz serial-servo tests
        )
        for frame in sorted(vinfo_options["frames"].keys()):
            self.start_subtest("Testing frame (%s)" % str(frame))
            if frame in known_broken_frames:
                self.progress("Skipping known-broken frame (%s)" %
                              (known_broken_frames[frame]))
                continue
            frame_bits = vinfo_options["frames"][frame]
            if frame_bits.get("external", False):
                self.progress("Skipping external simulation")
                continue
            model = frame_bits.get("model", frame)
            self.customise_SITL_commandline([], model=model, wipe=True)
            rc_reversed = frame in rc_pitch_reversed_frames
            if self.get_parameter('Q_ENABLE'):
                self.check_rc_input_direction_vtol(rc_pitch_reversed=rc_reversed)
            else:
                self.check_rc_input_direction_fixedwing(rc_pitch_reversed=rc_reversed)
        # we deliberately do not land each frame (some aerobatic frames do
        # not land cleanly, and the direction check does not need it) -
        # customise_SITL_commandline restarts SITL for the next frame.  Do
        # a final reset so the harness sees a disarmed vehicle at the end:
        self.reset_SITL_commandline()

    def check_rc_input_direction_fixedwing(self, rc_pitch_reversed=False):
        '''in FBWA, confirm pilot pitch/roll stick moves the aircraft in the
        expected direction (this is a sign check, not a magnitude one).
        rc_pitch_reversed inverts the expected pitch response for models
        that deliberately reverse the RC pitch input.'''
        # reset the sticks: an earlier frame leaves the throttle up, which
        # blocks arming ("Throttle (RC3) is not neutral") on the next one:
        self.set_rc_default()
        # climb using the autonomous TAKEOFF mode so the climb succeeds
        # regardless of the RC pitch direction under test:
        self.takeoff(alt=100, mode="TAKEOFF", timeout=120)
        self.change_mode('FBWA')
        self.set_rc(3, 2000)  # throttle up to hold flying speed
        self.wait_level_flight()

        # elevator back (high RC2) demands nose-up in FBWA (nose-down when
        # the RC pitch input is reversed):
        pitch_sign = -1 if rc_pitch_reversed else 1
        self.start_subtest("elevator stick pitches %s" %
                           ("down (RC pitch reversed)" if rc_pitch_reversed else "up"))
        self.set_rc(2, 2000)
        self.wait_pitch(pitch_sign * 18, accuracy=18)  # window [0, 36] or [-36, 0]
        self.set_rc(2, 1500)
        self.wait_level_flight()

        # aileron left (low RC1) demands a left bank (roll input is never
        # reversed on these frames):
        self.start_subtest("aileron stick banks left")
        self.set_rc(1, 1000)
        self.wait_roll(-30, accuracy=30)  # window [-60, 0]
        self.set_rc(1, 1500)
        self.wait_level_flight()

    def check_rc_input_direction_vtol(self, rc_pitch_reversed=False):
        '''in QHOVER, confirm pilot pitch/roll stick leans the vehicle in the
        expected direction (sign check).  rc_pitch_reversed inverts the
        expected pitch response for models that reverse RC pitch input.'''
        # reset the sticks: an earlier frame leaves the throttle up, which
        # blocks arming ("Throttle (RC3) is not neutral") on the next one:
        self.set_rc_default()
        self.takeoff(alt=20, mode='QHOVER', timeout=60)

        # elevator back (high RC2) leans the nose up (down when reversed):
        pitch_sign = -1 if rc_pitch_reversed else 1
        self.start_subtest("elevator stick pitches %s" %
                           ("down (RC pitch reversed)" if rc_pitch_reversed else "up"))
        self.set_rc(2, 1800)
        self.wait_pitch(pitch_sign * 12, accuracy=15)  # window [-3, 27] or [-27, 3]
        self.set_rc(2, 1500)
        self.wait_pitch(0, accuracy=12)  # settle before the roll check

        # aileron left (low RC1) leans left (roll input is never reversed):
        self.start_subtest("aileron stick banks left")
        self.set_rc(1, 1200)
        self.wait_roll(-12, accuracy=15)  # window [-27, 3]
        self.set_rc(1, 1500)
        self.wait_roll(0, accuracy=12)  # settle
