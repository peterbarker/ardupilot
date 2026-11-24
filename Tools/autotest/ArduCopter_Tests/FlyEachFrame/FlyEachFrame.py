'''
Copter autotest: FlyEachFrame

AP_FLAKE8_CLEAN
'''

import math

from pysim import vehicleinfo
from vehicle_test_suite import NotAchievedException

from test_discovery import TestGroup
from test_discovery import testcase


class FlyEachFrame(TestGroup):
    @testcase(bucket="CopterTests1c")
    def FlyEachFrame(self):
        '''Fly each supported internal frame'''
        vinfo = vehicleinfo.VehicleInfo()
        copter_vinfo_options = vinfo.options[self.vehicleinfo_key()]
        known_broken_frames = {
            'heli-compound': "wrong binary, different takeoff regime",
            'heli-dual': "wrong binary, different takeoff regime",
            'heli': "wrong binary, different takeoff regime",
            'heli-gas': "wrong binary, different takeoff regime",
            'heli-blade360': "wrong binary, different takeoff regime",
            'heli-ddvptail': "wrong binary, different takeoff regime",
            'heli-ddfptail': "wrong binary, different takeoff regime",
            'heli-quad': "wrong binary, different takeoff regime",
            "quad-can" : "needs CAN periph",
        }
        for frame in sorted(copter_vinfo_options["frames"].keys()):
            self.start_subtest("Testing frame (%s)" % str(frame))
            if frame in known_broken_frames:
                self.progress("Actually, no I'm not - it is known-broken (%s)" %
                              (known_broken_frames[frame]))
                continue
            frame_bits = copter_vinfo_options["frames"][frame]
            print("frame_bits: %s" % str(frame_bits))
            if frame_bits.get("external", False):
                self.progress("Actually, no I'm not - it is an external simulation")
                continue
            model = frame_bits.get("model", frame)
            self.context_push()
            frame_script = frame_bits.get('frame_example_script', None)
            if frame_script is not None:
                self.install_example_script_context(frame_script)
            self.customise_SITL_commandline(
                [],
                model=model,
                wipe=True,
            )
            if frame_script is not None:
                self.set_parameters({
                    "SCR_ENABLE": 1,
                    "LOG_BITMASK": 65535,
                })
                self.reboot_sitl()

            # add a listener that verifies yaw looks good:
            def verify_yaw(mav, m):
                if m.get_type() != 'ATTITUDE':
                    return
                yawspeed_thresh_rads = math.radians(20)
                if m.yawspeed > yawspeed_thresh_rads:
                    raise NotAchievedException("Excessive yaw on takeoff: %f deg/s > %f deg/s (frame=%s)" %
                                               (math.degrees(m.yawspeed), math.degrees(yawspeed_thresh_rads), frame))
            self.context_push()
            self.install_message_hook_context(verify_yaw)
            self.takeoff(10)
            self.context_pop()
            self.hover()
            self.change_mode('ALT_HOLD')
            self.delay_sim_time(1, reason="attitude to stabilise in alt_hold")

            def verify_rollpitch(mav, m):
                if m.get_type() != 'ATTITUDE':
                    return
                pitch_thresh_rad = math.radians(2)
                if m.pitch > pitch_thresh_rad:
                    raise NotAchievedException("Excessive pitch %f deg > %f deg" %
                                               (math.degrees(m.pitch), math.degrees(pitch_thresh_rad)))
                roll_thresh_rad = math.radians(2)
                if m.roll > roll_thresh_rad:
                    raise NotAchievedException("Excessive roll %f deg > %f deg" %
                                               (math.degrees(m.roll), math.degrees(roll_thresh_rad)))
            self.context_push()
            self.install_message_hook_context(verify_rollpitch)
            for i in range(5):
                self.set_rc(4, 2000)
                self.delay_sim_time(0.5, reason="yaw input to take effect")
                self.set_rc(4, 1500)
                self.delay_sim_time(5, reason="attitude to settle after yaw")
            self.context_pop()

            self.do_RTL()

            self.context_pop()
